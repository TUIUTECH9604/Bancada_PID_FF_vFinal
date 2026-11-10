# Ponte NetworkTables + scripts de teste da bancada

Ferramentas de linha de comando (Git Bash, Windows) para ler e escrever valores no robô ao vivo,
rodar os testes de controle da bancada e analisar os resultados. Foram usadas para ajustar o
`DialSubsystem` em 2026-11-07.

## Instalação (uma vez)

Precisa do WPILib 2026 instalado (em `C:\Users\Public\wpilib\2026`) e de Python 3 com `numpy`
(só para o `sysid_fit.py`).

```bash
cd tools/ntbridge
./setup.sh        # copia as bibliotecas do WPILib para libs/ e compila a ponte
```

`libs/`, `out/` e os `.csv` não vão para o git: o `setup.sh` recria tudo.

Para outro IP de robô: `export NT_SERVER=10.96.4.2` (o padrão é `169.254.29.48`, a bancada por cabo).
Para o WPILib em outra pasta: `export WPILIB_HOME=...` antes do `setup.sh`.

## Ponte

```bash
./nt.sh list /Tuning                        # tópicos e valores
./nt.sh get /Tuning/Dial/kP Robot/m_robotContainer/dial/getDeg
./nt.sh set /Tuning/Dial/kP 0.003
./nt.sh rec 10 dados.csv Robot/m_robotContainer/dial   # grava 10 s (t,topico,valor)
./nt.sh enabled && echo habilitado
```

Os tópicos do Epilogue ficam **sem barra inicial**: `Robot/m_robotContainer/dial/...`.

## Testes (o ponteiro se MOVE — robô habilitado em Teleop)

Todos usam o comando remoto `/Tuning/Dial/RemoteCmd`, que só age com o robô habilitado e volta a 0
ao desabilitar. **Disable para tudo na hora.**

| Script | O que faz | Saída |
|---|---|---|
| `./kdtest.sh kP kD arq.csv` | degraus SEM perfil 0° → 60° → 0° | overshoot, erro final, tremida |
| `./proftest.sh kP kD arq.csv` | os mesmos degraus COM perfil | idem |
| `./holdtest.sh arq.csv` | perfil até 60° e segura 12 s | erro e tremida a cada 2 s |
| `./motiontest.sh vel accel kA arq.csv` | perfil 0° → 90° → 0° com limites dados | erro de seguimento, overshoot |

Atenção: `kdtest`/`proftest` deixam o kP/kD testado aplicado; `motiontest` deixa os limites do
perfil e o kA. Restaure os valores (ou reinicie o código) ao terminar.

Valores de referência da bancada (ponteiro ~400 g): kP 0,003, kD 0,00012 → overshoot ≤ 1°,
erro final ~0,2°, tremida 0,1°.

## Testes finos (os que validaram os ganhos atuais)

| Script | O que faz |
|---|---|
| `./robusto.sh arq.csv` | **o teste de referência**: passos de 15° e movimentos de 90°/180°, como o controle. **Parada automática** se o ponteiro oscilar (4+ inversões de sentido >0,7° em 1 s, duas vezes seguidas). No fim imprime o resumo ou "ABORTADO (instavel)" |
| `./sweep.sh nome "kD=0.00015 kI=0.01"` | aplica parâmetros ao vivo (`/Tuning/Dial/...`) e roda o `robusto.sh` → `rb_nome.csv` |
| `./mapa.sh arq.csv` | passos de 15° de 0° a 180° e de volta (sem movimentos grandes) |
| `python mapa_ana.py arq.csv` | análise ângulo a ângulo; `-r arq1.csv arq2.csv ...` só o resumo |

O resumo mostra: **tremor rápido** (vibração > ~5 Hz, graus rms), **pp** (quanto mexe parado),
**assenta** (tempo até parar de se mexer, pela quadratura), **overshoot** e **erro final**.
Referência atual: tremor 0,002–0,005, pp 0,08–0,17, assenta 0,6–0,8 s, overshoot 0,6–0,8°.

Lições: valide com movimentos grandes (só passos pequenos aprovaram ganhos que oscilavam no uso
real) e repita cada configuração (o atrito faz variar muito entre rodadas).

## SysId (kS, kG, kV, kA)

1. Robô habilitado, ponteiro pendurado e parado, área livre (ele dá voltas a até ~200 RPM).
2. `./nt.sh set /Tuning/Dial/RemoteCmd 9` e espere `/SysId/Status` = "Concluído" (~1 min).
3. `./nt.sh set /Tuning/Dial/RemoteCmd 0`
4. `./sysid_get.sh` — baixa `/home/lvuser/sysid.csv` e ajusta o modelo
   `V = kS·sinal(v) + kG·cos θ + kV·v + kA·a`.

O script mostra dois métodos: **direto** e **discreto** (o mesmo da ferramenta SysId do WPILib).
Na bancada, o kA do método discreto (0,0000786) foi o que funcionou melhor ao vivo.
