/**
 * 직접 모터에 목표 각도 한 번만 보내는 테스트
 * ros2_control 없이 모터 내부 보간만 사용
 */

#include <iostream>
#include <myactuator_rmd/actuator_interface.hpp>
#include <myactuator_rmd/driver/can_driver.hpp>

int main(int argc, char** argv)
{
  std::string ifname = "can0";
  
  // 목표 각도 (degree)
  float target1 = 0.0f;
  float target2 = 0.0f;
  float target3 = 0.0f;
  float target4 = 0.0f;
  float speed = 30.0f;  // deg/s
  
  if (argc >= 5) {
    target1 = std::stof(argv[1]);
    target2 = std::stof(argv[2]);
    target3 = std::stof(argv[3]);
    target4 = std::stof(argv[4]);
  }
  if (argc >= 6) {
    speed = std::stof(argv[5]);
  }
  
  std::cout << "=== Direct Position Test ===" << std::endl;
  std::cout << "Target positions (deg): " << target1 << ", " << target2 << ", " << target3 << ", " << target4 << std::endl;
  std::cout << "Speed: " << speed << " deg/s" << std::endl;
  
  try {
    myactuator_rmd::CanDriver driver(ifname);
    
    myactuator_rmd::ActuatorInterface motor1(driver, 1);
    myactuator_rmd::ActuatorInterface motor2(driver, 2);
    myactuator_rmd::ActuatorInterface motor3(driver, 3);
    myactuator_rmd::ActuatorInterface motor4(driver, 4);
    
    std::cout << "Sending position commands..." << std::endl;
    
    motor1.sendPositionAbsoluteSetpoint(target1, speed);
    motor2.sendPositionAbsoluteSetpoint(target2, speed);
    motor3.sendPositionAbsoluteSetpoint(target3, speed);
    motor4.sendPositionAbsoluteSetpoint(target4, speed);
    
    std::cout << "Done! Motors moving with internal interpolation." << std::endl;
    
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }
  
  return 0;
}
