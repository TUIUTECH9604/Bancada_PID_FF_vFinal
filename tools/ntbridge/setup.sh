#!/bin/bash
# Copia as bibliotecas do NetworkTables da instalação do WPILib 2026 e compila a ponte.
set -e
D="$(cd "$(dirname "$0")" && pwd)"
W="${WPILIB_HOME:-/c/Users/Public/wpilib/2026}"
M="$W/maven"; V=2026.2.1
cd "$D"; mkdir -p libs out
cp "$M/edu/wpi/first/ntcore/ntcore-java/$V/ntcore-java-$V.jar" libs/ntcore-java.jar
cp "$M/edu/wpi/first/wpiutil/wpiutil-java/$V/wpiutil-java-$V.jar" libs/wpiutil-java.jar
cp "$M/edu/wpi/first/wpinet/wpinet-java/$V/wpinet-java-$V.jar" libs/wpinet-java.jar
cp "$M/us/hebi/quickbuf/quickbuf-runtime/1.4/quickbuf-runtime-1.4.jar" libs/quickbuf-runtime.jar
for j in core databind annotations; do
  cp "$M/com/fasterxml/jackson/core/jackson-$j/2.19.2/jackson-$j-2.19.2.jar" libs/jackson-$j.jar
done
for z in ntcore wpinet wpiutil; do
  unzip -o -j -q "$M/edu/wpi/first/$z/$z-cpp/$V/$z-cpp-$V-windowsx86-64.zip" "windows/x86-64/shared/*.dll" -d libs
done
"$W/jdk/bin/javac" -encoding UTF-8 -d out -cp "libs/ntcore-java.jar;libs/wpiutil-java.jar;libs/wpinet-java.jar" NtBridge.java
echo "Pronto. Teste com: ./nt.sh list /Tuning"
