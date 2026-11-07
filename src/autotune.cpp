#include "autotune.h"
#include "sensors.h"
#include "motors.h"
#include "encoders.h"
#include "telemetry.h"
#include "navigator.h"
#include <Preferences.h>

static Preferences mapPrefs;

static TuneState currentTuneState = TUNE_IDLE;
static bool isFig8 = false;
static bool mapValid = false;
static bool gainTableValid = false;

//crawling data buffers (up to 30m track at 2.5cm/bin)
static constexpr uint16_t MAX_CRAWL_BINS = 1200;
static constexpr float BIN_CM = 2.5f;

static int16_t crawlDTheta[MAX_CRAWL_BINS]; //milli-radians
static uint16_t totalCrawlBins = 0;
static uint16_t lockedLapBins = 0;

static int32_t crawlStartLeft = 0;
static int32_t crawlStartRight = 0;
static int32_t prevBinLeft = 0;
static int32_t prevBinRight = 0;

static float netHeadingRad = 0.0f;
static float crawlX = 0.0f;
static float crawlY = 0.0f;
static uint16_t candLapBins = 0;
static bool overlapSearching = false;

//pivot calibration state
static uint8_t pivotPhase = 0;
static int32_t pivotStartL = 0;
static int32_t pivotStartR = 0;

//spatial sensor contrast profiling
static uint16_t floorGlarePeak[NUM_SENSORS];
static uint16_t tapeSignalFloor[NUM_SENSORS];

//packets
static MapPathPacket g_mapPath = {};
static MapSegmentsPacket g_mapSegs = {};
static TuneStatusPacket g_tuneStatus = {};
static GainTablePacket g_gainTable = {};

//progressive tuning state
static uint8_t currentTier = 0;
static uint8_t currentTrial = 0;
static uint8_t totalTiers = 8;
static uint8_t lapCounter = 0;
static int32_t lapStartTicks = 0;
static uint16_t currentSegIdx = 0;

//metrics collection
static uint32_t sampleCount = 0;
static int64_t straightErrSqSum = 0;
static uint32_t straightSampleCount = 0;
static int64_t curveErrSqSum = 0;
static uint32_t curveSampleCount = 0;
static int32_t maxErrMagnitude = 0;
static int32_t prevCorrection = 0;
static int64_t oscDeltaSum = 0;
static uint8_t peakLiftDuringLap = 0;

//soft-save reflex state
static bool softSaveActive = false;
static uint32_t softSaveStartTime = 0;

//probed tuning gains for current trial
static float testKp = PID_KP;
static float testKd = PID_KD;
static int16_t testStraightPwm = 90;
static int16_t testCornerPwm = 80;
static float trialCostWinner = 999999.0f;
static float trialWinnerKp = PID_KP;
static float trialWinnerKd = PID_KD;

//speed tiers configuration
static const int16_t SPEED_TIER_STRAIGHT[8] = { 90, 110, 130, 150, 170, 190, 210, 230 };
static const int16_t SPEED_TIER_CORNER[8]   = { 75,  85,  95, 105, 115, 125, 135, 140 };

//velocity regulation for crawl
static uint32_t lastVelCheckMs = 0;
static int32_t lastVelAvgTicks = 0;
static int16_t crawlBasePwm = CRAWL_BASE_PWM;

static void extractSegments() {
    if (lockedLapBins == 0) return;

    g_mapSegs.header = 0xB2;
    uint8_t segIdx = 0;
    uint16_t startBin = 0;

    auto getBinCurv = [](uint16_t b) -> float {
        return (static_cast<float>(crawlDTheta[b]) * 0.001f) / BIN_CM; //rad/cm
    };

    while (startBin < lockedLapBins && segIdx < 18) {
        float curvStart = getBinCurv(startBin);
        uint8_t type = 0; //straight
        if (curvStart > 0.008f) {
            type = (curvStart > 0.035f) ? 3 : 1; //sharp left or gentle left
        } else if (curvStart < -0.008f) {
            type = (curvStart < -0.035f) ? 4 : 2; //sharp right or gentle right
        }

        uint16_t endBin = startBin + 1;
        float curvSum = curvStart;

        while (endBin < lockedLapBins) {
            float c = getBinCurv(endBin);
            uint8_t nextType = 0;
            if (c > 0.008f) nextType = (c > 0.035f) ? 3 : 1;
            else if (c < -0.008f) nextType = (c < -0.035f) ? 4 : 2;

            if (nextType != type) break;
            curvSum += c;
            endBin++;
        }

        uint16_t segLenBins = endBin - startBin;
        float segLenCm = segLenBins * BIN_CM;
        float avgCurv = curvSum / segLenBins;
        int16_t radCm = (fabsf(avgCurv) > 0.002f) ? static_cast<int16_t>(1.0f / avgCurv) : 0;

        g_mapSegs.segs[segIdx].type = type;
        g_mapSegs.segs[segIdx].startCm = static_cast<uint16_t>(startBin * BIN_CM);
        g_mapSegs.segs[segIdx].lengthCm = static_cast<uint16_t>(segLenCm);
        g_mapSegs.segs[segIdx].radiusCm = radCm;
        g_mapSegs.segs[segIdx].maxSafePwm = (type == 0) ? 220 : ((abs(radCm) < 25) ? 95 : 125);

        segIdx++;
        startBin = endBin;
    }

    g_mapSegs.numSegments = segIdx;
    g_mapSegs.checksum = calcProtocolChecksum(reinterpret_cast<const uint8_t*>(&g_mapSegs), sizeof(g_mapSegs) - 1);
}

static void buildMapPathPacket() {
    g_mapPath.header = 0xB1;
    g_mapPath.totalLengthCm = static_cast<uint16_t>(lockedLapBins * BIN_CM);
    g_mapPath.windingSign = (netHeadingRad > 1.0f) ? 1 : ((netHeadingRad < -1.0f) ? -1 : 0);
    g_mapPath.hasGaps = 0;
    g_mapPath.numPoints = 120;

    //downsample crawlBins into 120 evenly spaced points
    float step = static_cast<float>(lockedLapBins) / 120.0f;
    for (uint8_t i = 0; i < 120; i++) {
        uint16_t b = constrain(static_cast<uint16_t>(i * step), 0, lockedLapBins - 1);
        float dTh = static_cast<float>(crawlDTheta[b]) * 0.001f;
        int16_t scaled = static_cast<int16_t>(dTh * 250.0f);
        g_mapPath.dThetaScaled[i] = static_cast<int8_t>(constrain(scaled, -127, 127));
    }

    //calculate recommended optical parameters
    uint16_t maxGlare = 0;
    uint16_t minTape = 4095;
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        if (floorGlarePeak[i] > maxGlare) maxGlare = floorGlarePeak[i];
        if (tapeSignalFloor[i] < minTape && tapeSignalFloor[i] > 100) minTape = tapeSignalFloor[i];

        //equalize channel sensitivity based on noise floor
        if (floorGlarePeak[i] > 300) {
            //high glare sensor: slightly reduce sensitivity to raise trigger threshold
            g_mapPath.optSens[i] = 85;
        } else if (tapeSignalFloor[i] < 600) {
            //weak tape reflection: boost sensitivity
            g_mapPath.optSens[i] = 115;
        } else {
            g_mapPath.optSens[i] = 100;
        }
    }

    int16_t recThresh = (maxGlare + minTape) / 2;
    g_mapPath.optThresh = constrain(recThresh, 350, 650);
    g_mapPath.optAccelSlew = 3;

    g_mapPath.checksum = calcProtocolChecksum(reinterpret_cast<const uint8_t*>(&g_mapPath), sizeof(g_mapPath) - 1);
}

void autotuneInit() {
    currentTuneState = TUNE_IDLE;
    mapValid = false;
    gainTableValid = false;

    //initialize default gain table
    g_gainTable.header = 0xB4;
    g_gainTable.numTiers = 8;
    for (uint8_t i = 0; i < 8; i++) {
        g_gainTable.tiers[i].straightPwm = SPEED_TIER_STRAIGHT[i];
        g_gainTable.tiers[i].cornerPwm   = SPEED_TIER_CORNER[i];
        g_gainTable.tiers[i].kpX10000    = static_cast<int16_t>((0.038f + i * 0.005f) * 10000.0f);
        g_gainTable.tiers[i].kdX1000     = static_cast<int16_t>((0.220f + i * 0.020f) * 1000.0f);
        g_gainTable.tiers[i].rmsErr01Mm  = 0;
        g_gainTable.tiers[i].qualityScore= 0;
        g_gainTable.tiers[i].state       = 0; //pending
    }
    g_gainTable.checksum = calcProtocolChecksum(reinterpret_cast<const uint8_t*>(&g_gainTable), sizeof(g_gainTable) - 1);

    //attempt to load previous map from nvs
    if (mapPrefs.begin("lfr_map", true)) {
        if (mapPrefs.isKey("map_valid")) {
            mapPrefs.getBytes("map_path", &g_mapPath, sizeof(g_mapPath));
            mapPrefs.getBytes("map_segs", &g_mapSegs, sizeof(g_mapSegs));
            mapPrefs.getBytes("gain_lut", &g_gainTable, sizeof(g_gainTable));
            lockedLapBins = static_cast<uint16_t>(g_mapPath.totalLengthCm / BIN_CM);
            mapValid = true;
            gainTableValid = true;
            Serial.println("//loaded saved track map and gain schedule from nvs");
        }
        mapPrefs.end();
    }
}

void autotuneStartCrawl(bool figure8Mode) {
    isFig8 = figure8Mode;
    currentTuneState = TUNE_PIVOT_CAL;
    pivotPhase = 0;
    pivotStartL = getLeftTicks();
    pivotStartR = getRightTicks();
    sensorsResetCalibrationMinMax();

    totalCrawlBins = 0;
    lockedLapBins = 0;
    netHeadingRad = 0.0f;
    crawlX = 0.0f;
    crawlY = 0.0f;
    candLapBins = 0;
    overlapSearching = false;
    crawlBasePwm = CRAWL_BASE_PWM;
    lastVelCheckMs = millis();
    lastVelAvgTicks = getAvgTicks();

    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        floorGlarePeak[i] = 0;
        tapeSignalFloor[i] = 1000;
    }

    motorsArm(true);
}

void autotuneMarkLapNow() {
    if (currentTuneState == TUNE_CRAWL_MAP && totalCrawlBins > 20) {
        lockedLapBins = totalCrawlBins;
        extractSegments();
        buildMapPathPacket();
        mapValid = true;
        currentTuneState = TUNE_WAIT_VALIDATE;
        drive(0, 0);
        motorsArm(false);
        Serial.printf("//manual lap mark accepted: %d bins (%.1f cm)\n", lockedLapBins, lockedLapBins * BIN_CM);
    }
}

void autotuneValidateAndStartSweep() {
    if (currentTuneState != TUNE_WAIT_VALIDATE && !mapValid) return;

    currentTuneState = TUNE_SPEED_SWEEP;
    currentTier = 0;
    currentTrial = 0;
    lapCounter = 0;
    lapStartTicks = getAvgTicks();
    sampleCount = 0;
    straightErrSqSum = 0;
    straightSampleCount = 0;
    curveErrSqSum = 0;
    curveSampleCount = 0;
    maxErrMagnitude = 0;
    oscDeltaSum = 0;
    peakLiftDuringLap = 0;

    testStraightPwm = SPEED_TIER_STRAIGHT[0];
    testCornerPwm   = SPEED_TIER_CORNER[0];
    testKp = PID_KP;
    testKd = PID_KD;
    trialCostWinner = 999999.0f;
    trialWinnerKp = testKp;
    trialWinnerKd = testKd;

    motorsArm(true);
}

void autotuneSaveToNVS() {
    if (!mapPrefs.begin("lfr_map", false)) return;
    mapPrefs.putBytes("map_path", &g_mapPath, sizeof(g_mapPath));
    mapPrefs.putBytes("map_segs", &g_mapSegs, sizeof(g_mapSegs));
    mapPrefs.putBytes("gain_lut", &g_gainTable, sizeof(g_gainTable));
    mapPrefs.putBool("map_valid", true);
    mapPrefs.end();

    //also write recommended optical settings to lfr_cfg
    setSensorThreshold(g_mapPath.optThresh);
    setAllSensorSensitivities(g_mapPath.optSens);
    setMotorAccelSlew(g_mapPath.optAccelSlew);
    saveConfigToNVS();

    telemetrySendConfig(ACK_MAP_SAVED);
    Serial.println("//saved track map and auto-tuned schedule to nvs flash");
}

void autotuneRequestSync() {
    if (mapValid) {
        telemetryBroadcastMapPath();
        telemetryBroadcastMapSegments();
    }
    if (gainTableValid) {
        telemetryBroadcastGainTable();
    }
}

TuneState getTuneState() {
    return currentTuneState;
}

const TuneStatusPacket& getTuneStatus() {
    return g_tuneStatus;
}

const MapPathPacket& getMapPath() {
    return g_mapPath;
}

const MapSegmentsPacket& getMapSegments() {
    return g_mapSegs;
}

const GainTablePacket& getGainTable() {
    return g_gainTable;
}

bool isMapValid() {
    return mapValid;
}

bool isGainTableValid() {
    return gainTableValid;
}

//500hz update loop
void autotuneUpdate() {
    int32_t currentTicks = getAvgTicks();
    int32_t leftEnc = getLeftTicks();
    int32_t rightEnc = getRightTicks();

    //update live telemetry status packet
    g_tuneStatus.header = 0xB3;
    g_tuneStatus.tuneState = static_cast<uint8_t>(currentTuneState);
    g_tuneStatus.currentTier = currentTier;
    g_tuneStatus.totalTiers = totalTiers;
    g_tuneStatus.subTrial = currentTrial;
    g_tuneStatus.lapNumber = lapCounter;
    g_tuneStatus.currentSegIdx = static_cast<uint8_t>(currentSegIdx);
    g_tuneStatus.currentDistCm = static_cast<uint16_t>(ticksToCm(currentTicks - crawlStartLeft));
    g_tuneStatus.totalLengthCm = static_cast<uint16_t>(lockedLapBins * BIN_CM);
    g_tuneStatus.netHeadingDeg = static_cast<int16_t>(netHeadingRad * (180.0f / PI));
    g_tuneStatus.straightPwm = testStraightPwm;
    g_tuneStatus.cornerPwm = testCornerPwm;
    g_tuneStatus.activeKp = testKp;
    g_tuneStatus.activeKd = testKd;
    g_tuneStatus.lastLiftPeak = peakLiftDuringLap;
    g_tuneStatus.statusFlags = (mapValid ? 0x01 : 0) | (gainTableValid ? 0x02 : 0) | (softSaveActive ? 0x04 : 0);
    g_tuneStatus.checksum = calcProtocolChecksum(reinterpret_cast<const uint8_t*>(&g_tuneStatus), sizeof(g_tuneStatus) - 1);

    switch (currentTuneState) {
        case TUNE_PIVOT_CAL: {
            //pivot left-right over line to sample contrast without lifting bot
            sensorsSampleCalibrationStep();
            if (pivotPhase == 0) {
                //pivot left until outer left sensors see black
                driveDirect(-75, 75);
                if (isSensorBlack(2) || isSensorBlack(3) || (leftEnc - pivotStartL < -120)) {
                    pivotPhase = 1;
                }
            } else if (pivotPhase == 1) {
                //pivot right until outer right sensors see black
                driveDirect(75, -75);
                if (isSensorBlack(12) || isSensorBlack(13) || (rightEnc - pivotStartR < -120)) {
                    pivotPhase = 2;
                }
            } else if (pivotPhase == 2) {
                //return to center
                driveDirect(-70, 70);
                if (isSensorBlack(7) || isSensorBlack(8)) {
                    driveDirect(0, 0);
                    sensorsFinalizeCalibration();
                    crawlStartLeft = leftEnc;
                    crawlStartRight = rightEnc;
                    prevBinLeft = leftEnc;
                    prevBinRight = rightEnc;
                    currentTuneState = TUNE_CRAWL_MAP;
                }
            }
            break;
        }

        case TUNE_CRAWL_MAP: {
            //regulate forward crawl speed to ~14 cm/s
            if (millis() - lastVelCheckMs >= 30) {
                float dt = (millis() - lastVelCheckMs) * 0.001f;
                lastVelCheckMs = millis();
                float distDeltaCm = ticksToCm(currentTicks - lastVelAvgTicks);
                lastVelAvgTicks = currentTicks;
                float currentSpeedCmS = distDeltaCm / dt;

                //adjust base pwm to hold target crawl speed
                float speedErr = CRAWL_SPEED_CM_S - currentSpeedCmS;
                crawlBasePwm = constrain(crawlBasePwm + static_cast<int16_t>(speedErr * 0.6f), 65, 120);
            }

            //steer along line at slow crawl speed
            int32_t error = computePositionError(false);
            int16_t correction = static_cast<int16_t>(g_cfg.pidKp * error);
            drive(constrain(crawlBasePwm + correction, 0, 255), constrain(crawlBasePwm - correction, 0, 255));

            //check bin progression (every 2.5cm)
            int32_t binDistTicks = (currentTicks - (prevBinLeft + prevBinRight) / 2);
            if (binDistTicks >= cmToTicks(BIN_CM)) {
                int32_t dL = leftEnc - prevBinLeft;
                int32_t dR = rightEnc - prevBinRight;
                prevBinLeft = leftEnc;
                prevBinRight = rightEnc;

                float ds = (dL + dR) * 0.5f / TICKS_PER_CM;
                float dTheta = (dR - dL) / (TICKS_PER_CM * WHEEL_BASE_CM);

                if (totalCrawlBins < MAX_CRAWL_BINS) {
                    crawlDTheta[totalCrawlBins] = static_cast<int16_t>(dTheta * 1000.0f);
                    totalCrawlBins++;
                }

                netHeadingRad += dTheta;
                crawlX += ds * cosf(netHeadingRad);
                crawlY += ds * sinf(netHeadingRad);

                //profile spatial sensor contrast
                for (uint8_t i = 0; i < NUM_SENSORS; i++) {
                    int16_t rawNorm = getSensorNormalizedDark(i);
                    if (isSensorBlack(i)) {
                        if (rawNorm < tapeSignalFloor[i]) tapeSignalFloor[i] = rawNorm;
                    } else {
                        if (rawNorm > floorGlarePeak[i]) floorGlarePeak[i] = rawNorm;
                    }
                }

                //automatic loop closure test
                float totalDistCm = totalCrawlBins * BIN_CM;
                if (totalDistCm > 120.0f) {
                    float headingDeg = netHeadingRad * (180.0f / PI);
                    bool headingClosed = isFig8 ? (fabsf(headingDeg) < 35.0f) : (fabsf(headingDeg) >= 285.0f && fabsf(headingDeg) <= 435.0f);
                    float distFromOrigin = sqrtf(crawlX * crawlX + crawlY * crawlY);

                    if (headingClosed && distFromOrigin < (totalDistCm * 0.22f)) {
                        if (!overlapSearching) {
                            candLapBins = totalCrawlBins;
                            overlapSearching = true;
                        }
                    }

                    if (overlapSearching) {
                        uint16_t overlapBins = totalCrawlBins - candLapBins;
                        uint16_t targetOverlap = max<uint16_t>(10, candLapBins / 4);

                        if (overlapBins >= targetOverlap) {
                            //cross correlate curvature to lock exact loop length
                            uint16_t bestL = candLapBins;
                            float minDiff = 9999.0f;

                            for (int16_t testL = candLapBins - 8; testL <= candLapBins + 8; testL++) {
                                if (testL <= 0 || testL >= totalCrawlBins) continue;
                                float sumDiff = 0.0f;
                                for (uint16_t k = 0; k < overlapBins; k++) {
                                    uint16_t idxA = totalCrawlBins - 1 - k;
                                    int32_t idxB = calcCorrelationPairIdx(totalCrawlBins, k, testL);
                                    if (idxB < 0) continue;
                                    sumDiff += fabsf(static_cast<float>(crawlDTheta[idxA] - crawlDTheta[idxB]));
                                }
                                if (sumDiff < minDiff) {
                                    minDiff = sumDiff;
                                    bestL = testL;
                                }
                            }

                            lockedLapBins = bestL;
                            extractSegments();
                            buildMapPathPacket();
                            mapValid = true;
                            currentTuneState = TUNE_WAIT_VALIDATE;
                            drive(0, 0);
                            motorsArm(false);
                            Serial.printf("//auto loop closed: %d bins (%.1f cm)\n", lockedLapBins, lockedLapBins * BIN_CM);
                        }
                    }
                }
            }
            break;
        }

        case TUNE_WAIT_VALIDATE: {
            drive(0, 0);
            motorsArm(false);
            break;
        }

        case TUNE_SPEED_SWEEP: {
            //handle soft-save reflex if line drift reached outer wing
            if (softSaveActive) {
                if (millis() - softSaveStartTime < 350) {
                    //pull back onto line with low forward speed
                    int32_t err = computePositionError(false);
                    int16_t steer = constrain(static_cast<int16_t>(testKp * err * 1.5f), -120, 120);
                    drive(75 + steer, 75 - steer);
                    return;
                } else {
                    softSaveActive = false;
                }
            }

            //line following with progressive tier speed
            int32_t error = computePositionError(false);
            if (abs(error) >= SOFT_SAVE_ERR_01MM) {
                //soft-save trigger!
                softSaveActive = true;
                softSaveStartTime = millis();
                //record that this speed exceeded safe cornering grip
                if (currentSegIdx < g_mapSegs.numSegments) {
                    g_mapSegs.segs[currentSegIdx].maxSafePwm = testCornerPwm - 10;
                }
                return;
            }

            //steer along track
            int16_t correction = static_cast<int16_t>(testKp * error);
            int16_t targetSpeed = (abs(error) > 2000) ? testCornerPwm : testStraightPwm;
            drive(constrain(targetSpeed + correction, 0, 255), constrain(targetSpeed - correction, 0, 255));

            //accumulate lap metrics
            sampleCount++;
            int32_t absErr = abs(error);
            if (absErr > maxErrMagnitude) maxErrMagnitude = absErr;

            uint8_t lift = getLastFrameLift();
            if (lift > peakLiftDuringLap) peakLiftDuringLap = lift;

            oscDeltaSum += abs(correction - prevCorrection);
            prevCorrection = correction;

            if (abs(error) > 1800) {
                curveErrSqSum += static_cast<int64_t>(error) * error;
                curveSampleCount++;
            } else {
                straightErrSqSum += static_cast<int64_t>(error) * error;
                straightSampleCount++;
            }

            //check lap completion (using encoder distance modulo lockedLapBins)
            int32_t lapDistTicks = currentTicks - lapStartTicks;
            if (lapDistTicks >= cmToTicks(lockedLapBins * BIN_CM)) {
                lapStartTicks = currentTicks;
                lapCounter++;

                //calculate lap metrics
                float straightRms = straightSampleCount > 0 ? sqrtf(static_cast<float>(straightErrSqSum / straightSampleCount)) * 0.001f : 0.0f;
                float curveRms = curveSampleCount > 0 ? sqrtf(static_cast<float>(curveErrSqSum / curveSampleCount)) * 0.001f : 0.0f;
                float oscIndex = sampleCount > 0 ? static_cast<float>(oscDeltaSum) / sampleCount : 0.0f;
                float lapCost = 1.0f * curveRms + 1.2f * straightRms + 0.35f * (maxErrMagnitude * 0.001f) + 0.7f * oscIndex;

                if (lapCost < trialCostWinner) {
                    trialCostWinner = lapCost;
                    trialWinnerKp = testKp;
                    trialWinnerKd = testKd;
                }

                //sub-trial state progression (0: baseline, 1: Kp probe, 2: Kd probe)
                if (currentTrial == 0) {
                    //probe Kp
                    currentTrial = 1;
                    if (curveRms > straightRms * 1.3f) {
                        testKp *= 1.15f; //understeer in corners: raise Kp
                    } else {
                        testKp *= 0.90f; //wobble: lower Kp
                    }
                } else if (currentTrial == 1) {
                    //probe Kd
                    currentTrial = 2;
                    testKp = trialWinnerKp;
                    testKd *= 1.20f; //probe damping
                } else {
                    //tier complete! lock winner
                    g_gainTable.tiers[currentTier].straightPwm  = testStraightPwm;
                    g_gainTable.tiers[currentTier].cornerPwm    = testCornerPwm;
                    g_gainTable.tiers[currentTier].kpX10000     = static_cast<int16_t>(trialWinnerKp * 10000.0f);
                    g_gainTable.tiers[currentTier].kdX1000      = static_cast<int16_t>(trialWinnerKd * 1000.0f);
                    g_gainTable.tiers[currentTier].rmsErr01Mm   = static_cast<uint16_t>(straightRms * 10.0f);
                    g_gainTable.tiers[currentTier].qualityScore = static_cast<uint8_t>(constrain(100 - static_cast<int>(trialCostWinner * 2.0f), 10, 100));
                    g_gainTable.tiers[currentTier].state        = 2; //locked
                    g_gainTable.checksum = calcProtocolChecksum(reinterpret_cast<const uint8_t*>(&g_gainTable), sizeof(g_gainTable) - 1);
                    gainTableValid = true;

                    currentTier++;
                    currentTrial = 0;
                    trialCostWinner = 999999.0f;

                    if (currentTier >= totalTiers || SPEED_TIER_STRAIGHT[currentTier] > g_cfg.maxSpeed) {
                        currentTuneState = TUNE_COMPLETE;
                        drive(0, 0);
                        motorsArm(false);
                        telemetryBroadcastGainTable();
                        Serial.println("//progressive auto-tune complete across all tiers");
                        return;
                    }

                    //seed next tier
                    testStraightPwm = SPEED_TIER_STRAIGHT[currentTier];
                    testCornerPwm   = SPEED_TIER_CORNER[currentTier];
                    testKp = trialWinnerKp + 0.004f;
                    testKd = trialWinnerKd + 0.015f;
                    trialWinnerKp = testKp;
                    trialWinnerKd = testKd;
                }

                //reset metric accumulators for next lap
                sampleCount = 0;
                straightErrSqSum = 0;
                straightSampleCount = 0;
                curveErrSqSum = 0;
                curveSampleCount = 0;
                maxErrMagnitude = 0;
                oscDeltaSum = 0;
                peakLiftDuringLap = 0;
            }
            break;
        }

        case TUNE_COMPLETE:
        case TUNE_SAFE_ABORT:
        default:
            drive(0, 0);
            break;
    }
}

//500hz mapped race update with pre-corner braking and gain scheduling
void mappedRaceUpdate() {
    if (!mapValid || lockedLapBins == 0 || g_mapSegs.numSegments == 0) {
        drive(0, 0);
        return;
    }

    int32_t currentTicks = getAvgTicks();
    int32_t lapDistTicks = wrapLapTicks(currentTicks, cmToTicks(lockedLapBins * BIN_CM));
    float currentDistCm = ticksToCm(lapDistTicks);

    //find current segment
    uint8_t segIdx = 0;
    for (uint8_t i = 0; i < g_mapSegs.numSegments; i++) {
        if (currentDistCm >= g_mapSegs.segs[i].startCm &&
            currentDistCm < (g_mapSegs.segs[i].startCm + g_mapSegs.segs[i].lengthCm)) {
            segIdx = i;
            break;
        }
    }
    currentSegIdx = segIdx;

    //lookahead to next curve for braking envelope
    float targetPwm = g_mapSegs.segs[segIdx].maxSafePwm;
    uint8_t nextSeg = (segIdx + 1) % g_mapSegs.numSegments;
    if (g_mapSegs.segs[nextSeg].type != 0) { //next is curve
        float distToCurve = (g_mapSegs.segs[nextSeg].startCm > currentDistCm) ?
            (g_mapSegs.segs[nextSeg].startCm - currentDistCm) :
            (g_mapPath.totalLengthCm - currentDistCm + g_mapSegs.segs[nextSeg].startCm);

        float curveSpeed = g_mapSegs.segs[nextSeg].maxSafePwm;
        float safeEntry = curveSpeed + distToCurve * BRAKE_DECEL_PWM_PER_CM;
        if (targetPwm > safeEntry) {
            targetPwm = safeEntry;
        }
    }

    //interpolate PID gains from gain table
    float curKp = g_cfg.pidKp;
    float curKd = g_cfg.pidKd;
    if (gainTableValid) {
        for (int i = 7; i >= 0; i--) {
            if (targetPwm >= g_gainTable.tiers[i].straightPwm) {
                curKp = g_gainTable.tiers[i].kpX10000 * 0.0001f;
                curKd = g_gainTable.tiers[i].kdX1000  * 0.001f;
                break;
            }
        }
    }

    static int32_t prevMappedError = 0;
    int32_t error = computePositionError(false);
    int32_t dErrRaw = constrain(error - prevMappedError, -1500, 1500);
    float dError = static_cast<float>(dErrRaw) / (LOOP_INTERVAL_US * 1e-6f);
    prevMappedError = error;
    int16_t correction = static_cast<int16_t>(curKp * error + curKd * dError * 0.001f);
    drive(constrain(static_cast<int16_t>(targetPwm) + correction, 0, 255), constrain(static_cast<int16_t>(targetPwm) - correction, 0, 255));
}
