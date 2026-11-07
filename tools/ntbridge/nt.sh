#!/bin/bash
# Ponte NetworkTables (NT4) para o robô, pela linha de comando (Git Bash).
#   ./nt.sh list [prefixo]                     tópicos e valores
#   ./nt.sh get topico...                      lê
#   ./nt.sh set topico valor                   escreve
#   ./nt.sh rec segundos arquivo.csv prefixo...   grava mudanças (t_robô,topico,valor)
#   ./nt.sh enabled                            código de saída 0 se o robô estiver habilitado
# Servidor: variável NT_SERVER (padrão 169.254.29.48, o IP da bancada por cabo).
# Antes do primeiro uso: ./setup.sh
D="$(cd "$(dirname "$0")" && pwd)"
JP="${WPILIB_JDK:-/c/Users/Public/wpilib/2026/jdk}/bin"
cd "$D" || exit 1
[ -d out ] || { echo "Rode ./setup.sh primeiro"; exit 1; }
export MSYS_NO_PATHCONV=1
if [ "$1" = "enabled" ]; then
  v=$("$0" get /FMSInfo/FMSControlData | awk '{print $3}')
  [[ "$v" =~ ^[0-9]+$ ]] && (( v & 1 )) && exit 0 || exit 1
fi
CP="out;libs/ntcore-java.jar;libs/wpiutil-java.jar;libs/wpinet-java.jar;libs/quickbuf-runtime.jar;libs/jackson-core.jar;libs/jackson-databind.jar;libs/jackson-annotations.jar"
PATH="$D/libs:$PATH" "$JP/java" -Djava.library.path=libs -Dnt.server="${NT_SERVER:-169.254.29.48}" \
  -cp "$CP" NtBridge "$@" 2>&1 | grep -v "^NT:"
