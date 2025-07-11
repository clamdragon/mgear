rem mkdir "build"
rem cd "cmake"
rem cmake -G "Visual Studio 16 2019" -A x64 -T v141 -D MAYA_VERSION=2020 ..\
cmake -G "Visual Studio 17 2022" -A x64 -T v143 -D MAYA_VERSION=2025 ../
cmake --build . --config Release
cmd /k