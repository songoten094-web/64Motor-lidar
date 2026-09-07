# CMake generated Testfile for 
# Source directory: D:/Dynamic/LivoxHmi/native
# Build directory: D:/Dynamic/LivoxHmi/native/build-win-x64
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
if(CTEST_CONFIGURATION_TYPE MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
  add_test(LivoxHmiBridgeAbiCheck "D:/Dynamic/LivoxHmi/native/build-win-x64/Debug/LivoxHmiBridgeAbiCheck.exe")
  set_tests_properties(LivoxHmiBridgeAbiCheck PROPERTIES  _BACKTRACE_TRIPLES "D:/Dynamic/LivoxHmi/native/CMakeLists.txt;44;add_test;D:/Dynamic/LivoxHmi/native/CMakeLists.txt;0;")
elseif(CTEST_CONFIGURATION_TYPE MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
  add_test(LivoxHmiBridgeAbiCheck "D:/Dynamic/LivoxHmi/native/build-win-x64/Release/LivoxHmiBridgeAbiCheck.exe")
  set_tests_properties(LivoxHmiBridgeAbiCheck PROPERTIES  _BACKTRACE_TRIPLES "D:/Dynamic/LivoxHmi/native/CMakeLists.txt;44;add_test;D:/Dynamic/LivoxHmi/native/CMakeLists.txt;0;")
elseif(CTEST_CONFIGURATION_TYPE MATCHES "^([Mm][Ii][Nn][Ss][Ii][Zz][Ee][Rr][Ee][Ll])$")
  add_test(LivoxHmiBridgeAbiCheck "D:/Dynamic/LivoxHmi/native/build-win-x64/MinSizeRel/LivoxHmiBridgeAbiCheck.exe")
  set_tests_properties(LivoxHmiBridgeAbiCheck PROPERTIES  _BACKTRACE_TRIPLES "D:/Dynamic/LivoxHmi/native/CMakeLists.txt;44;add_test;D:/Dynamic/LivoxHmi/native/CMakeLists.txt;0;")
elseif(CTEST_CONFIGURATION_TYPE MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
  add_test(LivoxHmiBridgeAbiCheck "D:/Dynamic/LivoxHmi/native/build-win-x64/RelWithDebInfo/LivoxHmiBridgeAbiCheck.exe")
  set_tests_properties(LivoxHmiBridgeAbiCheck PROPERTIES  _BACKTRACE_TRIPLES "D:/Dynamic/LivoxHmi/native/CMakeLists.txt;44;add_test;D:/Dynamic/LivoxHmi/native/CMakeLists.txt;0;")
else()
  add_test(LivoxHmiBridgeAbiCheck NOT_AVAILABLE)
endif()
subdirs("livox_sdk2")
