cmake -B build -S . -G "Visual Studio 18 2026" -DQt6_DIR="D:\dev\Qt\6.11.1\msvc2022_64\lib\cmake\Qt6" -DQT_DIR="D:\dev\Qt\6.11.1\msvc2022_64\lib\cmake\Qt6" -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="D:\bin\vgmtrans"
cmake --build build --parallel 8 --preset win-x64-release
