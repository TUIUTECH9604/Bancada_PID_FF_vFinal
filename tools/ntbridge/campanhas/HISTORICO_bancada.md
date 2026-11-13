# Histórico das campanhas da bancada (antes ficavam em comentários no bateria.sh)

Mecanismo `bancada` (ponteiro, NEO 15:1). Detalhes e números no CLAUDE.md.

| # | Pasta | O que testou | Resultado |
|---|---|---|---|
| 8 | `bat_20261112kd` | kD 0,0003 / 0,00015 / 0,000075 (D parado 0,3) | **kD 0,00015**: grandes 0,86/1,01° (era 1,9–2,7°). Ver `bancada_kd.sh`. |
| 7 | `bat_20261112dp` | DParadoFator 0,3 / 0 | 0,3 (com 1 oscilava 9/10 em 270°). |
| 6 | `bat_20261112af` | AntiFolga 0 / 1 (8°, fator 0,7) | 60>75 4,1 → 2,9°, 45>60 sem ganho: ficou desligada. |
| 5 | `bat_20261112kg` | kG 0,063 / 0,09 / 0,123 (ponteiro com parafusos) | A 5 ms quase iguais; ficou 0,09. |
| 4 | `bat_20261111e` | perfil 720/4000 com kA 0 / 0,00005 / 0,0001 / 0,0002 | **a05** (kA 0,00005). |
| 3 | `bat_20261111d` | perfil 360/2000, 540/3000, 720/4000, 720/6000 | passa 0,35° (360) → 1,2° (720) → 2,0° (720/6000). |
| 2 | `bat_20261111b` | ganhos ×1 / ×1,5 / ×2 (kP 0,003 base) | **×2**: kP 0,006 kI 0,02 kD 0,0003. |
| 1 | `bat_20261111` | ×1, kA do SysId, ×2 com kA, ×3,5 com kA | ×3,5 instável (10 Hz); kA do SysId atrapalhou. |
| 0 | `bat_20261110` | sem redução, +220 g: base, kD 20/10, kP 40, kI 05, zm 08, kA 91 | kA 0,000106, kD 0,0001, kP 0,004 rejeitados. |
