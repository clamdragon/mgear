mkdir "build"
cd "build"
cmake -G "Visual Studio 16 2019" -A x64 -T v141 -D MAYA_VERSION=2020 ..\
cmake --build . --config Release --target Install
cmd /k