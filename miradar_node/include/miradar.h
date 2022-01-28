#ifndef MIRADAR_H_
#define MIRADAR_H_
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <iostream>
#include <iterator>
#include <string>
#include <vector>

constexpr uint8_t CR = 0x0d;
constexpr uint8_t LF = 0x0a;
constexpr int COMM_RX_BYTE_UNIT = 64;

struct PPIData {
    int distance;
    int angle;
    int speed;
    int db;
};

struct MiRadarParam {
    explicit MiRadarParam()
        : minDistance(300),
          maxDistance(3000),
          alarmDistance(0),
          nDistance(64),
          nAngle(45),
          maxAngle(44),
          txPower(-7),
          minDb(-40),
          maxDb(-20),
          hpfGain(1),
          pgaGain(1),
          duration(100) {}

    int minDistance;
    int maxDistance;
    int alarmDistance;
    int nDistance;
    int nAngle;
    int maxAngle;
    int minAngle;
    int txPower;
    int minDb;
    int maxDb;
    int hpfGain;
    int pgaGain;
    int duration;
};

class Serial {
   public:
    int fd;

    ~Serial() { close(fd); }

    int CommInit(std::string deviceFile) {
        fd = open(deviceFile.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        setupSerial();
        if (fd < 0) {
            return (-1);
        }
        return (fd);
    }

    void setupSerial() {
        struct termios tio;
        bzero((void*)&tio, (size_t)(sizeof(tio)));
        tio.c_cflag = B921600 | CS8 | CLOCAL | CREAD;
        tio.c_iflag = IGNPAR;
        tio.c_oflag = 0;
        tio.c_lflag = 0;
        tio.c_cc[VTIME] = 0;
        tio.c_cc[VMIN] = 1;

        tcflush(fd, TCIFLUSH);
        tcsetattr(fd, TCSANOW, &tio);
        fcntl(fd, F_SETFL, FNDELAY);
    }

    int CommTx(char* bpBuf, int nLen) {
        int nErr;
        nErr = write(fd, bpBuf, nLen);
        printf("TX %dbyte : %s", nLen, bpBuf);
        return (nErr);
    }
    int CommRx(char* bpBuf, int nBufSize) {
        char* bp1;
        int nErr;
        int nRxLen = 0;
        int nNopCnt = 0;
        int nReqQuit = 0;
        memset(bpBuf, 0, nBufSize);
        bp1 = bpBuf;

        while (nReqQuit == 0) {
            nErr = -1;
            while (nErr < 0) {
                nErr = read(fd, bp1, COMM_RX_BYTE_UNIT);
                //------- received
                if (0 < nErr) {
                    nNopCnt = 0;
                    bp1 += nErr;
                    nRxLen += nErr;
                    if (nBufSize <= (nRxLen + COMM_RX_BYTE_UNIT)) {
                        nErr = -1;
                        nReqQuit = 1;
                        break;
                    }
                    continue;
                }
                //------- no received
                usleep(1000);
                nNopCnt++;
                if ((0 < nRxLen) && (10 < nNopCnt)) {
                    nReqQuit = 1;
                    break;
                }
                if (1000 < nNopCnt) {
                    nReqQuit = 1;
                    break;
                }
            }
        }

        return (nRxLen);
    }
};

std::vector<std::string> split(std::string str, char del) {
    int first = 0;
    int last = str.find_first_of(del);

    std::vector<std::string> result;

    while (first < str.size()) {
        std::string subStr(str, first, last - first);

        result.push_back(subStr);

        first = last + 1;
        last = str.find_first_of(del, first);

        if (last == std::string::npos) {
            last = str.size();
        }
    }

    return result;
}

char* toCharArray(std::string str) {
    char* chrArrC = reinterpret_cast<char*>(malloc(str.size() + 1));
    strcpy(chrArrC, str.c_str());
    return chrArrC;
}

class MiRadar {
public:
    std::vector<PPIData> ppiEntries;
    Serial comm;
    char sRxBuf[65516];
    std::vector<uint8_t> map;
    int nDistance;
    int nAngle;
    int sensorState = 0;
    int prevState = 0;
    MiRadarParam radarParam;
    char sCmd[9] = {'A', ',', '0', ',', '1', ',', '0', 0x0d, 0x0a};

    explicit MiRadar() : sensorState(1) {
        radarParam.maxDistance = 3000;
        radarParam.minDistance = 300;
        radarParam.nDistance = 64;
        radarParam.alarmDistance = 0;
        radarParam.maxAngle = 44;
        radarParam.nAngle = 45;
        radarParam.txPower = -7;
        radarParam.hpfGain = 1;
        radarParam.pgaGain = 1;
        radarParam.maxDb = -20;
        radarParam.minDb = -40;
        radarParam.duration = 200;
    }

    static double pixel2DB(int pix) {
        return static_cast<double>(pix) * 0.25 - 73.75;
    }

    void setSensorState(int state) {
        if (state <= 2 && state >= 0) {
            sCmd[4] = '0' + state;
            sensorState = state;
        }
    }

    void sendSensorMode() {
        comm.CommTx(sCmd, sizeof(sCmd));
        comm.CommRx(sRxBuf, sizeof(sRxBuf));
    }

    void stopCommunication() {
        std::string stopCommand = "A,0,0";
        stopCommand.push_back(static_cast<char>(CR));
        stopCommand.push_back(static_cast<char>(LF));
        comm.CommTx(&stopCommand[0], sizeof(stopCommand.c_str()));
        comm.CommRx(sRxBuf, sizeof(sRxBuf));
    }

    void setDistance(int minDistance, int maxDistance) {
        if (minDistance > 0 && maxDistance > 0) {
            std::string distanceCommand = "B,0," + std::to_string(maxDistance) +
                                          "," + std::to_string(minDistance);
            distanceCommand.push_back((char)CR);
            distanceCommand.push_back((char)LF);

            comm.CommTx(&distanceCommand[0], sizeof(distanceCommand.c_str()));
            comm.CommRx(sRxBuf, sizeof(sRxBuf));
        }
    }

    void setAlarm(int distance) {
        if (distance > 0) {
            std::string alarmCommand = "C,0," + std::to_string(distance);
            alarmCommand.push_back((char)CR);
            alarmCommand.push_back((char)LF);
            comm.CommTx(&alarmCommand[0], sizeof(alarmCommand.c_str()));
            comm.CommRx(sRxBuf, sizeof(sRxBuf));
        }
    }

    void setParam() {
        sensorState = (sensorState == 0) ? 1 : sensorState;
        std::string paramCommand =
            "A,0," + std::to_string(sensorState + 0x10) + ",";

        paramCommand += std::to_string(radarParam.maxDistance) + ",";
        paramCommand += std::to_string(radarParam.minDistance) + ",";
        paramCommand += std::to_string(radarParam.alarmDistance) + ",";
        paramCommand += std::to_string(radarParam.nDistance) + ",";
        paramCommand += std::to_string(radarParam.maxAngle) + ",";
        paramCommand += std::to_string(radarParam.nAngle) + ",";
        paramCommand += std::to_string(radarParam.txPower) + ",";
        paramCommand += std::to_string(radarParam.hpfGain) + ",";
        paramCommand += std::to_string(radarParam.pgaGain) + ",";
        paramCommand += std::to_string(radarParam.minDb) + ",";
        paramCommand += std::to_string(radarParam.duration);
        paramCommand.push_back((char)CR);
        paramCommand.push_back((char)LF);
        std::cout << paramCommand << std::endl;

        char* commandCstr = toCharArray(paramCommand);
        comm.CommTx(&paramCommand[0], sizeof(paramCommand.c_str()));
        comm.CommRx(sRxBuf, sizeof(sRxBuf));
    }

    void setParam(MiRadarParam param) {
        if (param.minDb < param.maxDb && param.hpfGain > 0 &&
            param.pgaGain > 0 && param.nAngle > 0 && param.nDistance > 0) {
            if (prevState != sensorState) {
                setSensorState(sensorState);
                sendSensorMode();
            }

            std::cout << sensorState << std::endl;
            std::cout << std::to_string(sensorState + 16) << std::endl;
            std::string paramCommand =
                "A,0," + std::to_string(sensorState + 0x10) + ",";
            paramCommand += std::to_string(param.maxDistance) + ",";
            paramCommand += std::to_string(param.minDistance) + ",";
            paramCommand += std::to_string(param.alarmDistance) + ",";
            paramCommand += std::to_string(param.nDistance) + ",";
            paramCommand += std::to_string(param.maxAngle) + ",";
            paramCommand += std::to_string(param.nAngle) + ",";
            paramCommand += std::to_string(param.txPower) + ",";
            paramCommand += std::to_string(param.hpfGain) + ",";
            paramCommand += std::to_string(param.pgaGain) + ",";
            paramCommand += std::to_string(param.minDb) + ",";
            paramCommand += std::to_string(param.duration);
            paramCommand.push_back((char)CR);
            paramCommand.push_back((char)LF);
            std::cout << paramCommand << std::endl;
        } else {
            printParam(param);
            std::cout << "parameter is wrong." << std::endl;
        }
    }

    void printParam(MiRadarParam param) {
        std::cout << "min distance : " << param.minDistance;
        std::cout << " max distance : " << param.maxDistance;
        std::cout << " duration : " << param.duration;
        std::cout << " min db : " << param.minDb;
        std::cout << " max db : " << param.maxDb;
        std::cout << " max angle : " << param.maxAngle;
        std::cout << " angle div : " << param.nAngle;
        std::cout << " distance div : " << param.nDistance << std::endl;
    }

    void setSerial(Serial& ser) { comm = ser; }

    char* getReceivedBuffer() { return sRxBuf; }
    std::string logcommand;

    std::string logcommand1;
    void run() {
        if (sensorState == 0) {
            return;
        }
        int size = comm.CommRx(sRxBuf, sizeof(sRxBuf));
        std::string receivedBytes(sRxBuf, size);
        std::cout << receivedBytes << std::endl;
        if (sensorState == 1) {
            ppiEntries.clear();

            if (receivedBytes.find("M,1") != -1 ||
                receivedBytes.find("M,0") != -1) {
                std::vector<std::string> metadata = split(receivedBytes, ',');
                int entrynumbers = (metadata.size()) / 4;
                if (entrynumbers > 2) {
                    metadata.erase(metadata.begin());
                    metadata.erase(metadata.begin());
                }
                entrynumbers = (metadata.size()) / 4;

                for (int j = 0; j < entrynumbers; j++) {
                    if (std::stoi(metadata[4 * j]) != 0 ||
                        std::stoi(metadata[4 * j + 1]) != 0 ||
                        std::stoi(metadata[4 * j + 2]) != 0) {
                        PPIData ppidata;
                        ppidata.distance = std::stoi(metadata[4 * j]);
                        ppidata.angle = std::stoi(metadata[4 * j + 1]);
                        ppidata.speed = std::stoi(metadata[4 * j + 2]);
                        ppidata.db = std::stoi(metadata[4 * j + 3]);
                        ppiEntries.push_back(ppidata);
                    }
                }
            }
        } else if (sensorState == 2) {
            if (receivedBytes.find("BEGIN_MAP_") != -1) {
                map.clear();
                int endIndex = receivedBytes.find("END_MAP");
                if (endIndex != -1) {
                    std::string mapStr =
                        receivedBytes.substr(18, endIndex - 18);
                    std::string header = receivedBytes.substr(0, 18);
                    std::string nDistanceStr;
                    std::string nAngleStr;
                    for (int i = 0; i < 3; i++) {
                        if (header[10 + i] != '0' || i == 2) {
                            nDistanceStr.push_back(header[10 + i]);
                        }
                        if (header[14 + i] != '0' || i == 2) {
                            nAngleStr.push_back(header[14 + i]);
                        }
                    }
                    nDistance = std::stoi(nDistanceStr);
                    nAngle = std::stoi(nAngleStr);

                    if (nDistance * nAngle == endIndex - header.size()) {
                        std::copy(mapStr.begin(), mapStr.end(),
                                  std::back_inserter(map));
                    } else {
                        std::cout << "map is corrupt" << std::endl;
                        std::cout << "map size is " << header.size()
                                  << " , requested size is " << nDistance << " "
                                  << nAngle << std::endl;
                    }
                }
            }
        }
        prevState = sensorState;
    }
};

#endif
