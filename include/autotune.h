#pragma once
#include <Arduino.h>
#include "protocol.h"
#include "config.h"

void autotuneInit();
void autotuneStartCrawl(bool figure8Mode);
void autotuneMarkLapNow();
void autotuneValidateAndStartSweep();
void autotuneSaveToNVS();
void autotuneRequestSync();

//500hz update loop during MODE_MAP_AND_TUNE
void autotuneUpdate();

//500hz update loop during MODE_MAPPED_RACE
void mappedRaceUpdate();

TuneState getTuneState();
const TuneStatusPacket& getTuneStatus();
const MapPathPacket& getMapPath();
const MapSegmentsPacket& getMapSegments();
const GainTablePacket& getGainTable();
bool isMapValid();
bool isGainTableValid();
