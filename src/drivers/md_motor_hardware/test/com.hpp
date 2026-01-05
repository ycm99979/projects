/**
 * ============================================================================
 * MD Motor Driver Communication Header
 * ============================================================================
 * 
 * 이 파일은 MD 듀얼채널 모터 드라이버와의 시리얼 통신을 위한 헤더 파일입니다.
 * 
 * [ros2_control 통합 관점]
 * ────────────────────────────────────────────────────────────────────────────
 * 이 코드는 "Hardware Interface"로 통합해야 합니다!
 * 
 * 이유:
 * 1. 이 코드는 실제 하드웨어(모터 드라이버)와 시리얼 통신을 담당
 * 2. 모터에 속도 명령을 전송하고, 엔코더 피드백을 수신
 * 3. ros2_control에서 Hardware Interface는 "실제 하드웨어와의 통신 계층"
 * 
 * [아키텍처 구조]
 * ┌─────────────────────────────────────────────────────────────────┐
 * │                        Controller Manager                        │
 * │  ┌──────────────────────┐  ┌─────────────────────────────────┐ │
 * │  │ diff_drive_controller │  │ joint_state_broadcaster        │ │
 * │  │ (Controller Interface)│  │ (Controller Interface)         │ │
 * │  └──────────┬───────────┘  └────────────┬────────────────────┘ │
 * │             │                           │                       │
 * │             ▼                           ▼                       │
 * │  ┌──────────────────────────────────────────────────────────┐  │
 * │  │              Hardware Interface (이 코드!)               │  │
 * │  │  - write(): 모터에 속도 명령 전송 (PutMdData)            │  │
 * │  │  - read(): 엔코더 값 수신 (ReceiveDataFromController)    │  │
 * │  └──────────────────────────────────────────────────────────┘  │
 * └─────────────────────────────────────────────────────────────────┘
 *                              │
 *                              ▼ 시리얼 통신 (RS-485)
 *                    ┌─────────────────────┐
 *                    │   MD Motor Driver   │
 *                    │   (듀얼 채널)        │
 *                    └─────────────────────┘
 * 
 * ============================================================================
 */

#ifndef MD_CONTROLLER_COM_HPP_
#define MD_CONTROLLER_COM_HPP_

#include <stdio.h>
#include <stdlib.h>
#include <sstream>
#include <iostream>
#include <float.h>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "serial/serial.h"
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include "std_msgs/msg/int16_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <tf2/LinearMath/Quaternion.h>

using namespace std;

/* ============================================================================
 * 상수 정의 (Constants)
 * ============================================================================ */

// 좌표축 인덱스
#define _X		          0       // X축 (전진/후진 방향)
#define _Y		          1       // Y축 (좌우 방향)
#define _THETA            2       // 세타 (회전각)
#define PI              3.14159265359

// 상태 플래그
#define ON                1       // 활성화
#define OFF               0       // 비활성화
#define RESET             0       // 리셋
#define FAIL              0       // 실패
#define SUCCESS           1       // 성공

// 절대값 매크로
#define Abs(a)            (((a)<(0)) ? -(a):(a))

/* ============================================================================
 * PID (Protocol ID) 정의 - MD 모터 드라이버 통신 프로토콜
 * ============================================================================
 * 
 * [ros2_control 통합 시 사용되는 주요 PID]
 * - PID_PNT_VEL_CMD (207): 듀얼 모터 속도 명령 → write() 함수에서 사용
 * - PID_MAIN_DATA (193): 엔코더 피드백 수신 → read() 함수에서 사용
 * ============================================================================ */

#define PID_REQ_PID_DATA    4     // PID 데이터 요청 (특정 PID의 데이터를 요청)
#define PID_TQ_OFF	        5     // 토크 OFF (모터 정지)
#define PID_COMMAND         10    // 일반 명령
#define PID_POSI_RESET      13    // 위치(엔코더) 리셋
#define PID_VEL_CMD         130   // 단일 모터 속도 명령
#define PID_MAIN_DATA       193   // 메인 데이터 (RPM, 위치, 전류 등 피드백)
#define PID_PNT_VEL_CMD     207   // 듀얼 채널 속도 명령 (좌/우 모터 동시 제어)
                                  // ★ ros2_control write()에서 핵심적으로 사용

/* ============================================================================
 * 패킷 크기 정의
 * ============================================================================ */
#define MAX_PACKET_SIZE     26    // 최대 패킷 크기 (바이트)
#define MAX_DATA_SIZE       23    // 최대 데이터 크기 (헤더 제외)

#define REQUEST_PNT_MAIN_DATA 2   // 메인 데이터 요청 플래그

#define DURATION            0.0001  // 통신 주기 (초)

// 타이밍 상수 (50us 기준 카운트)
#define TIME_50MS           1     // 50ms
#define TIME_100MS          2     // 100ms → ros2_control update rate에 맞춤
#define TIME_1S             20    // 1초
#define TIME_5S             100   // 5초

/* ============================================================================
 * 타입 정의 (Type Definitions)
 * ============================================================================ */
typedef unsigned char  BYTE;      // 1바이트 (0~255)
typedef unsigned short WORD;      // 2바이트 (0~65535)
typedef unsigned int   DWORD;     // 4바이트

/* ============================================================================
 * Communication 구조체
 * ============================================================================
 * 
 * MD 모터 드라이버와의 시리얼 통신 상태 및 데이터를 관리하는 구조체
 * 
 * [ros2_control 통합 시]
 * 이 구조체는 Hardware Interface 클래스의 멤버 변수로 포함됨
 * ============================================================================ */
typedef struct {
    // ─────────────────────────────────────────────────────────────────────
    // 송수신 버퍼 (Serial Buffers)
    // ─────────────────────────────────────────────────────────────────────
    BYTE bySndBuf[MAX_PACKET_SIZE];   // 송신 버퍼 (모터로 보낼 데이터)
    BYTE byRcvBuf[MAX_PACKET_SIZE];   // 수신 버퍼 (모터에서 받은 데이터)
    
    // ─────────────────────────────────────────────────────────────────────
    // 패킷 파싱 상태 (Packet Parsing State)
    // ─────────────────────────────────────────────────────────────────────
    BYTE byPacketSize;                // 현재 패킷 크기
    BYTE byPacketNum;                 // 패킷 내 현재 위치
    BYTE byIn, byStep;                // 파싱 단계 (상태 머신)
    BYTE byChkSend;                   // 송신 체크
    BYTE byChkRcv;                    // 수신 체크
    BYTE fgInIdleLine;                // 유휴 상태 플래그
    BYTE fgPacketOK;                  // 패킷 수신 완료 플래그
    BYTE fgComComple;                 // 통신 완료 플래그
    BYTE byTotalRcvDataNum;           // 총 수신 데이터 수
    BYTE fgChk;                       // 체크 플래그
    BYTE byChkSum;                    // 체크섬 값
    BYTE byMaxDataNum;                // 최대 데이터 수
    BYTE byDataNum;                   // 현재 데이터 수

    // ─────────────────────────────────────────────────────────────────────
    // 시리얼 포트 설정 (Serial Port Configuration)
    // [ros2_control] URDF의 <param>으로 설정 가능
    // ─────────────────────────────────────────────────────────────────────
    string nPort;                     // 시리얼 포트 (예: "/dev/ttyMotor")
    int nIDPC;                        // PC ID
    int nIDMDUI;                      // MDUI ID (184: UI 보드)
    int nIDMDT;                       // MDT ID (183: 모터 드라이버)
    int nRMID;                        // 원격 ID
    int nBaudrate;                    // 통신 속도 (예: 57600)
    int nWheelLength;                 // 바퀴 간격
    int fgDirSign;                    // 방향 부호
    short sSetDia;                    // 설정 직경
    short sSetWheelLen;               // 설정 바퀴 간격
    short sSetGear;                   // 설정 기어비
    int nCmdSpeed;                    // 명령 속도
    int nCmdAngSpeed;                 // 명령 각속도
    int nSlowstart;                   // 천천히 시작
    int nSlowdown;                    // 천천히 정지
    float nWheelDiameter;             // 바퀴 직경

    // ─────────────────────────────────────────────────────────────────────
    // 모터 피드백 데이터 (Motor Feedback Data)
    // ★ ros2_control read() 함수에서 이 값들을 state_interfaces로 export
    // ─────────────────────────────────────────────────────────────────────
    int rpm;                          // 현재 RPM (피드백)
    int position;                     // 현재 엔코더 위치 (피드백)

    // ─────────────────────────────────────────────────────────────────────
    // 통신 상태 플래그
    // ─────────────────────────────────────────────────────────────────────
    BYTE byChkComError;               // 통신 오류 카운트
    BYTE fgComDataChk;                // 데이터 수신 확인
    BYTE fgInitsetting;               // 초기화 완료 플래그

} Communication;
extern Communication Com;             // 전역 통신 객체

/* ============================================================================
 * MotorVar 구조체
 * ============================================================================
 * 
 * 모터 관련 변수 및 상태를 관리하는 구조체
 * 
 * [ros2_control 통합 시]
 * - PPR, Tick2RAD: 엔코더 → 라디안 변환에 사용
 * - position, rpm: state_interface로 export
 * ============================================================================ */
typedef struct {
    // ─────────────────────────────────────────────────────────────────────
    // 모터 설정 파라미터 (URDF <param>으로 설정)
    // ─────────────────────────────────────────────────────────────────────
    int ID;                           // 모터 드라이버 ID (1 또는 2)
    int GearRatio;                    // 기어비 (예: 25:1)
    int InitError;                    // 초기화 오류 카운트
    int poles;                        // 모터 극 수 (예: 8극)
    BYTE InitMotor;                   // 모터 초기화 상태

    // ─────────────────────────────────────────────────────────────────────
    // 모터 피드백 (★ ros2_control state_interface)
    // ─────────────────────────────────────────────────────────────────────
    short rpm;                        // 현재 RPM
    long position;                    // 현재 엔코더 위치

    // ─────────────────────────────────────────────────────────────────────
    // 엔코더 → 라디안 변환용 변수
    // ─────────────────────────────────────────────────────────────────────
    float current_tick;               // 현재 틱 값
    float last_diff_tick;             // 이전 틱 차이
    float last_tick;                  // 이전 틱 값
    float last_rad;                   // 이전 라디안 값 (누적 회전각)
    float Tick2RAD;                   // 틱 → 라디안 변환 계수
    float PPR;                        // Pulses Per Revolution (1회전당 펄스 수)
                                      // = poles * 3 * GearRatio
} MotorVar;
extern MotorVar Motor;                // 전역 모터 객체

/* ============================================================================
 * IByte 구조체 - 2바이트 데이터 분할/조합용
 * ============================================================================ */
typedef struct {
    BYTE byLow;                       // 하위 바이트 (LSB)
    BYTE byHigh;                      // 상위 바이트 (MSB)
} IByte;

/* ============================================================================
 * 함수 선언 (Function Declarations)
 * ============================================================================
 * 
 * [ros2_control Hardware Interface 매핑]
 * 
 * ┌─────────────────────────────────────────────────────────────────────────┐
 * │ Hardware Interface Method  │  현재 함수               │ 역할           │
 * ├─────────────────────────────────────────────────────────────────────────┤
 * │ on_init()                  │  InitSerial()            │ 시리얼 포트 열기│
 * │ on_configure()             │  (모터 초기화 로직)       │ 모터 설정      │
 * │ on_activate()              │  (토크 ON)               │ 모터 활성화    │
 * │ on_deactivate()            │  PID_TQ_OFF              │ 모터 비활성화  │
 * │ read()                     │  ReceiveDataFromController│ 엔코더 읽기   │
 * │ write()                    │  PutMdData(PID_PNT_VEL_CMD)│ 속도 명령    │
 * └─────────────────────────────────────────────────────────────────────────┘
 * ============================================================================ */

// ─────────────────────────────────────────────────────────────────────────────
// 바이트 변환 유틸리티 함수
// ─────────────────────────────────────────────────────────────────────────────
extern IByte Short2Byte(short sIn);                      // short → 2바이트 분할
extern int Byte2Short(BYTE byLow, BYTE byHigh);          // 2바이트 → short 조합
extern int Byte2LInt(BYTE byData1, BYTE byData2, 
                     BYTE byData3, BYTE byData4);        // 4바이트 → int 조합

// ─────────────────────────────────────────────────────────────────────────────
// 통신 초기화 (★ Hardware Interface on_init()에서 호출)
// ─────────────────────────────────────────────────────────────────────────────
extern int InitSerial(void);

// ─────────────────────────────────────────────────────────────────────────────
// 파라미터 초기화
// ─────────────────────────────────────────────────────────────────────────────
extern int InitSetParam(void);

// ─────────────────────────────────────────────────────────────────────────────
// 모터 명령 전송 (★ Hardware Interface write()에서 호출)
// - byPID: 프로토콜 ID (PID_PNT_VEL_CMD = 207 사용)
// - byID: 대상 장치 ID
// - id_num: 모터 ID
// - nArray: 전송할 데이터 배열
// ─────────────────────────────────────────────────────────────────────────────
extern int PutMdData(BYTE byPID, BYTE byID, int id_num, int nArray[]);

// ─────────────────────────────────────────────────────────────────────────────
// 수신 데이터 처리 (★ Hardware Interface read()에서 호출)
// ─────────────────────────────────────────────────────────────────────────────
extern int MdReceiveProc(void);                          // 수신 데이터 파싱 후 저장
extern int ReceiveDataFromController(BYTE init);         // 시리얼 데이터 수신
extern int AnalyzeReceivedData(BYTE byArray[], BYTE byBufNum);  // 패킷 분석

#endif  // MD_CONTROLLER_COM_HPP_
