# Diagnostico e plano de producao — drone_diagnostics / logging stack

Data: 2026-08-11/12 (diagnostico) + 2026-08-12 (remediacao). Escopo:
`ros_diagnostics/` (node + aggregator + InfluxDB logger) e `ulg_analysis/`
(InfluxDB + ulg_ingestor), rodando junto do simulador `uav_px4_simulator`.

## Status da remediacao (atualizado 2026-08-12)

Todos os 20 gaps abaixo foram corrigidos e validados ao vivo nesta sessao
de remediacao (plano completo em
`/home/saraiva/.claude/plans/indexed-puzzling-neumann.md`), com excecao dos
itens que sao inerentemente passos de campo (calibracao de thresholds
contra voo real — gap #19 — so pode ser feita apos os primeiros voos reais)
ou dependem de infraestrutura externa nao disponivel nesta sessao (destino
de backup fora do host, servidor de webhook real para os alertas —
implementados e testados em modo dry-run/local). Ver o resumo de commits/
diffs no `git status`/`git log` de `ros_diagnostics/`, `ulg_analysis/` e
`uav_px4_simulator/` para o detalhe de cada mudanca. Grafana (gap #17) foi
deliberadamente deixado fora de escopo por decisao do usuario (foco em
robustez/estabilidade, nao visualizacao).

Metodologia: validacao end-to-end com a simulacao PX4/Gazebo real rodando
(nao apenas leitura de codigo) — subimos os dois stacks, voamos o drone e
inspecionamos o InfluxDB e os logs dos containers. Um dos gaps abaixo (#1) foi
**reproduzido ao vivo duas vezes** durante este trabalho: o host reiniciou os
containers e o InfluxDB nao voltou sozinho nas duas ocasioes.

## Resumo executivo

O pipeline de diagnostics **funciona tecnicamente** (20 checks cobrindo os
principais subsistemas do PX4, schema de series temporais razoavel no
InfluxDB) mas **nao esta pronto para producao**: nao ha auto-recuperacao apos
falha/reboot, ha um segredo fraco versionado em texto puro, os logs de voo
brutos (`.ulg`) se perdem a cada execucao da simulacao, e nao existe nenhum
alerta — os dois stacks podem ficar quietamente quebrados por horas (ja
aconteceu, duas vezes, sem nenhuma acao humana).

## Gaps identificados

| # | Gap | Severidade | Evidencia |
|---|-----|------------|-----------|
| 1 | `influxdb` (em `ulg_analysis/docker-compose.yml`) nao tem `restart:` policy. Reboot/crash do host derruba o banco e ele **nao volta sozinho** — o `drone_diagnostics` (que tem `restart: unless-stopped`, mas roda num compose project separado, sem `depends_on` cruzando os dois) fica em loop de `Connection refused` indefinidamente. | **Critico** | Reproduzido ao vivo 2x nesta sessao: 7h de outage na primeira verificacao; o host reiniciou de novo durante este trabalho e o InfluxDB ficou `Exited (255)` ate eu subir manualmente. |
| 2 | Senha admin do InfluxDB hardcoded em texto puro (`adminpassword123`) em `ulg_analysis/docker-compose.yml:10`, **versionado no git** (commit `f9df2b6`). O token de API fica corretamente em `.env` (gitignored), mas a senha do usuario `admin` nao. | **Alto** | `git log` confirma o commit; arquivo esta em `git ls-files`. |
| 3 | Logs de voo `.ulg` do PX4 SITL **nunca chegam** ao pipeline de ingestao. `run.sh` sobe `px4_container` com `--rm` e nao monta o diretorio de trabalho do PX4 (`./log/AAAA-MM-DD/*.ulg`); `ulg_analysis/px4_logs/` so tem arquivos historicos aparentemente copiados a mao. Toda telemetria bruta de cada simulacao/voo se perde ao encerrar o container. | **Critico** (elimina o proposito do stack `ulg_analysis`) | `docker inspect px4_container` nao lista mount para `./log`; `ulg_ingestor` logou "Backfill: nothing new (all 126 files already ingested)" sem nenhum arquivo novo apos varios voos de teste. |
| 4 | Nenhum alerta ativo. Checks em ERROR sustentado (ex.: `PX4: Time Sync`, `PX4: Failsafe Flags`) ou falha de escrita no InfluxDB so aparecem se alguem abrir `rqt_robot_monitor` ou rodar uma query manual. | **Alto** | Confirmado por inspecao — nao ha Alertmanager/Grafana alerting/webhook em nenhum dos dois compose files. |
| 5 | `shared_folder/` (montado no simulador) acumula logs orfaos multi-GB sem rotacao — um arquivo de **73GB** (`simulation_restart2.log`), varios de 1-2GB, dono `root`. Sem `logrotate`/limpeza automatica isso enche o disco do host silenciosamente. | **Alto** | `ls -la shared_folder/` nesta sessao. |
| 6 | InfluxDB exposto em `0.0.0.0:8086` sem TLS e com o token de admin com escopo total, sem rotacao. `drone_diagnostics` roda com `network_mode: host` e `--privileged` no simulador — superficie de rede/host ampla para um ambiente que hoje so tem isolamento de rede local. | **Medio** | Leitura direta dos compose files. |
| 7 | Sem backup do volume `influxdb_data` (Docker named volume). Se o volume for removido (`docker compose down -v`, disco corrompido, etc.) todo o historico de diagnostics e flight logs ingeridos some — sem re-obtencao possivel dos `.ulg` originais (ver gap #3). | **Medio** | Nenhuma rotina de backup/export encontrada em nenhum dos repos. |
| 8 | `PX4: Time Sync` sempre reporta ERROR em SITL (offset de ~microssegundos vira a diferenca entre epoch do simulador e relogio real) — comportamento documentado como conhecido no `CLAUDE.md` do projeto, mas **nao ha supressao/anotacao** disso no aggregator; em producao isso mistura ruido conhecido com sinais reais, dificultando qualquer alerta futuro (gap #4) que dependa de "check X em ERROR = investigar". | **Baixo-Medio** | `CLAUDE.md` linha ~170; reconfirmado nos testes desta sessao. |
| 9 | Nenhum teste automatizado (CI) valida os 20 checks do `diagnostic_node.py` nem o schema de escrita do InfluxDB antes de merge — mudancas no node (ex.: renomear um campo, mudar um threshold) podem quebrar silenciosamente o pipeline de logging sem que ninguem perceba ate a proxima inspecao manual. | **Medio** | Nenhum workflow de CI encontrado nos repos; unico teste automatizado no repo Diagnostic e `ulg_analysis/tests/` (pytest de um otimizador de PID, sem relacao). |
| 10 | `diagnostics_logger_node.py` grava `level`/`level_numeric` mas descarta os campos numericos mais uteis para pos-morte de crash (`accel_vibration_metric_ms2`, `gyro_vibration_metric_rads`, `failure_detector_status`, `freefall`) por nao estarem na lista `_NUMERIC_KEYS`. Hoje so da pra saber que "IMU Vibration" ficou ERROR, nao o valor exato da vibracao no InfluxDB. | **Medio** (limita analise pos-incidente) | Leitura de `diagnostics_logger_node.py`; confirmado nos testes de crash desta sessao (secao de resultados abaixo). |
| 11 | **Bug confirmado**: a tag `level` gravada no InfluxDB para os pontos vindos de `/diagnostics_agg` fica sempre `"UNKNOWN"`, independente do nivel real do check. O valor correto (OK/WARN/ERROR) so existe no campo numerico `level_numeric` (0/1/2). Qualquer dashboard/alerta que filtre por `level` (tag) no InfluxDB nao funciona hoje — silenciosamente. | **Alto** (quebra qualquer alerta/dashboard baseado na tag) | Reproduzido ao vivo: query Flux por `check_name` retornando `level=UNKNOWN` em 100% dos pontos durante um voo real, enquanto `level_numeric` (`_value`) trazia 0/1/2 corretos. Causa provavel: o `diagnostic_aggregator` reescreve/normaliza o `DiagnosticStatus.level` do agregado de um jeito que o logger nao reconhece como OK/WARN/ERROR (ou o logger le o campo errado do agregado). Requer investigacao em `diagnostics_logger_node.py`/`diagnostic_aggregator` config. |
| 12 | O modo de voo padrao apos armar (`SYS_AUTOSTART=4005`) e **Position**, que exige `xy_valid=True`. Como o modelo Hermit nao tem GPS nem VIO ativo por padrao (`cs_gps=False`), qualquer arm (mesmo via `commander arm -f`) entra imediatamente em failsafe "blind land" a menos que o modo seja forcado para `Manual`/`Stabilized`/`Altctl` ANTES de armar. Isso tambem bloqueia `MAV_CMD_NAV_TAKEOFF` e setpoints de posicao via MAVLink ("Switching to Takeoff is currently not available"), inutilizando o fluxo de decolagem "padrao" documentado no README do simulador para este ambiente sem VIO. | **Critico** (bloqueia voo autonomo/QGC padrao) | Reproduzido ao vivo repetidas vezes nesta sessao; resolvido nos scripts de teste forcando `commander mode manual` antes do arm, mas isso e um workaround de teste, nao uma correcao do ambiente. |
| 13 | Instabilidade intermitente de timesync entre o bridge uXRCE-DDS e o PX4 (`WARN [timesync] time jump detected. Resetting time synchroniser.` recorrendo a cada poucos segundos) causou, em pelo menos duas execucoes desta sessao, falha de failsafe por perda de "datalink"/controle manual mesmo com comandos MAVLink sendo enviados a 20Hz — abortando testes de voo de forma nao deterministica. Ocorreu logo apos um reboot do host; pode ser jitter de CPU/scheduling ainda nao estabilizado ou um problema real de lockstep do SITL. | **Alto** (torna testes automatizados nao deterministicos) | Reproduzido ao vivo varias vezes; mitigado retentando o teste (nao resolvido na raiz). Recomendo investigar `chrony`/NTP do host, prioridade de CPU do processo PX4 SITL, e afinidade de nucleos antes de usar este ambiente em CI. |

## Evidencia ao vivo: o pipeline realmente pega um crash?

Sim — confirmado por dois eventos reais nesta sessao (nao simulados "de mentirinha"):

1. Uma subida instavel via `MANUAL_CONTROL` fez o drone acelerar descontroladamente
   e o log do PX4 fechou sozinho ("closed logfile") — consultando o InfluxDB na
   janela exata do evento: **`PX4: IMU Vibration` foi a ERROR varias vezes** e
   **`PX4: Vehicle Status` transicionou de WARN para ERROR e ficou assim** — ou
   seja, o pipeline pegou o evento anormal em tempo real, sem eu ter pedido.
2. O teste scriptado `crash_test_motor_cut.py` (corte de motor forcado em
   pleno voo, 8m de altura) produziu uma descida controlada-mas-forcada
   (~4 m/s de descida) que ficou em `PX4: Vehicle Status`=WARN e nao chegou a
   ERROR — a queda foi "suave" demais para cruzar o limiar de vibracao/EKF.
   Uma segunda tentativa a 25m para gerar um impacto mais forte esbarrou na
   instabilidade do gap #13 antes de completar.

Conclusao: o pipeline de logging **captura corretamente eventos anormais reais**
(gap zero aqui), mas os thresholds de severidade (`PX4: Vehicle Status`,
`PX4: IMU Vibration`) sao conservadores — um "crash" suave fica em WARN, nao
ERROR. Vale revisar se isso e desejado (ERROR deveria ser reservado para
falhas confirmadas de hardware) ou se os limiares devem ser mais sensiveis
para quedas.

## Plano de remediacao (ordenado por prioridade)

1. **(Critico, baixo esforco)** Adicionar `restart: unless-stopped` ao servico
   `influxdb` em `ulg_analysis/docker-compose.yml`, igualando `ulg_ingestor`.
   Nao resolve o outage entre os dois compose projects sozinho, mas garante
   sobrevivencia a reboot do host.
2. **(Critico, esforco medio)** Unificar orquestracao: ou (a) mover
   `drone_diagnostics` para o mesmo `docker-compose.yml` de `ulg_analysis` com
   `depends_on: influxdb: condition: service_healthy`, ou (b) se precisam
   ficar em stacks separados por motivo operacional, adicionar um
   healthcheck + `restart` agressivo no `drone_diagnostics` que reinicie o
   processo (nao so logue) quando `INFLUXDB_URL` estiver inacessivel por mais
   de N minutos, e subir os dois via um unico `systemd`/script de boot no
   host (ver item 4 abaixo).
3. **(Critico, esforco medio)** Persistir os `.ulg`: montar o diretorio de
   trabalho do PX4 dentro do `px4_container` (ou pelo menos `./log`) para um
   caminho no host, e apontar esse caminho para
   `ulg_analysis/px4_logs/<data>/` (via bind mount em `run.sh`, similar ao
   que ja e feito para `shared_folder`/`models`). Alternativa mais simples:
   um `trap` no `simulation.sh` que copia `./log/**/*.ulg` para
   `/root/shared_folder/px4_flight_logs/` (ja mapeado ao host) ao encerrar.
4. **(Alto, esforco baixo-medio)** Subir os dois stacks automaticamente no
   boot do host (systemd unit ou cron `@reboot` chamando
   `docker compose up -d` nos dois diretorios) — elimina a dependencia de
   alguem lembrar de religar manualmente apos reboot/crash.
5. **(Alto, esforco baixo)** Mover `DOCKER_INFLUXDB_INIT_PASSWORD` para
   `.env` (`INFLUXDB_ADMIN_PASSWORD`) e trocar o valor hardcoded por
   `${INFLUXDB_ADMIN_PASSWORD}`; gerar uma senha forte nova e revogar/rotacionar
   o token atual do `.env` (ja foi impresso em texto claro durante este
   trabalho de diagnostico — tratar como comprometido).
6. **(Alto, esforco medio)** Alertas minimos: um checador periodico (cron ou
   node ROS2 simples) que consulta `/diagnostics_agg` e/ou faz uma query Flux
   a cada X minutos e dispara um webhook (Slack/e-mail) se (a) nenhum ponto
   novo foi escrito no InfluxDB nos ultimos N minutos com o sim rodando, ou
   (b) um check ficar em ERROR por mais de N ciclos seguidos, excluindo o
   ruido conhecido do item 8.
7. **(Alto, esforco baixo)** `logrotate`/limpeza para `shared_folder/*.log` —
   comecar deletando/arquivando os arquivos multi-GB ja acumulados (pedir
   confirmacao antes, pois nao sei se ainda sao necessarios) e configurar
   rotacao por tamanho/idade daqui pra frente.
8. **(Medio, esforco baixo)** Anotar/filtrar o `PX4: Time Sync` como
   "conhecido ruidoso em SITL" no `diagnostics_config.yaml` (ex.: desabilitar
   via `enable_timesync: false` em ambiente de simulacao, ou criar um grupo
   separado no aggregator) para nao poluir os alertas do item 6.
9. **(Medio, esforco medio)** Backup periodico do volume `influxdb_data`
   (`influx backup` agendado, exportando para um destino fora do host) e/ou
   `INFLUXDB_RETENTION` mais conservador combinado com export para storage
   frio antes do expurgo.
10. **(Medio, esforco medio)** Adicionar ao `_NUMERIC_KEYS` de
    `diagnostics_logger_node.py` os campos `accel_vibration_metric_ms2`,
    `gyro_vibration_metric_rads`, `failure_detector_status` — sao os sinais
    mais valiosos para reconstruir um incidente depois, e hoje se perdem.
11. **(Medio, esforco medio)** CI minimo: workflow que builda o pacote ROS2,
    roda os testes de voo/crash descritos abaixo contra uma simulacao
    headless, e falha se os checks esperados nao aparecerem no InfluxDB —
    transforma os scripts de teste desta entrega em guarda-corpo permanente
    contra regressao silenciosa do pipeline de logging.

## Validacao end-to-end (o que foi testado nesta entrega)

Ver `uav_px4_simulator/shared_folder/flight_tests/` — script de voo nominal
(`flight_test.py`), scripts de crash intencional (`crash_test_motor_cut.py`,
`crash_test_collision.sh`), e o verificador contra o InfluxDB
(`verify_influx_window.py`). Resultados da execucao real (voo nominal limpo,
colisao real com `IMU Vibration` em ERROR capturado) estao documentados no
`README.md` daquela pasta.

---

## Addendum: o que falta especificamente para HARDWARE REAL (nao SITL)

Os gaps 1-13 acima valem tanto pra simulacao quanto pra producao. Esta secao
cobre o que muda — ou fica mais grave — quando o companion computer e um
drone real em voo, nao um container no laptop. Fonte: leitura linha a linha
de `diagnostics_logger_node.py` (nao so smoke test).

| # | Gap | Severidade | Detalhe |
|---|-----|------------|---------|
| 14 | **Bug de causa raiz encontrado**: em `diagnostics_logger_node.py`, a tag `level` usa `{0: "OK", 1: "WARN", 2: "ERROR"}.get(status.level, "UNKNOWN")` — mas `status.level` (campo ROS2 `uint8`) chega como **`bytes`** em rclpy (ex.: `b'\x00'`), nao `int`. O dict tem chaves `int`, entao a busca **nunca bate** e cai sempre no default `"UNKNOWN"`. Por isso o gap #11 acontece. O campo `level_numeric` funciona por acidente: usa `LEVEL_MAP`, construido com as constantes `DiagnosticStatus.OK/WARN/ERROR` (essas sim sao `bytes`, batendo com `status.level`). **Fix de uma linha**: `"level": {0: "OK", 1: "WARN", 2: "ERROR"}[LEVEL_MAP.get(status.level, 0)]` — reusar o lookup que ja funciona em vez de comparar `status.level` direto contra chaves `int`. | **Alto** (facil de corrigir, alta prioridade — desbloqueia qualquer dashboard/alerta por nivel) | `diagnostics_logger_node.py`, metodo `_cb_diag`. |
| 15 | **Buffer de escrita sem limite e so em RAM**: `self._pending` (fila de pontos aguardando escrita) nao tem tamanho maximo. Numa falha prolongada de rede/InfluxDB — cenario **normal** em campo (drone fora de alcance do link de telemetria, GCS desconectado, InfluxDB fora do ar) — a fila cresce indefinidamente a ~2Hz x N checks ate estourar a memoria do companion computer, podendo derrubar o processo (e potencialmente outros processos do stack de autonomia na mesma placa). Alem disso, por ser so RAM, um corte de energia (exatamente o cenario de crash que mais queremos logar) perde tudo que ainda nao foi escrito com sucesso. | **Critico para hardware real** (risco de OOM em voo + perda dos dados mais criticos, os do proprio crash) | Mesmo arquivo, `_pending`/`_flush`. Corrigir com: (a) cap no tamanho da fila, e (b) persistir a fila em disco local (SQLite/arquivo append-only) antes de tentar a rede — so remover do disco apos confirmacao de escrita no InfluxDB. Mesmo principio do `.ulg` do PX4 (gap #3): gravar local primeiro, sincronizar depois. |
| 16 | **Topologia de implantacao nao documentada**: nada nos dois repos diz se `drone_diagnostics`+InfluxDB devem rodar *a bordo* (companion computer) ou na *estacao em solo* (GCS). O comentario do compose ("reachable on the same LAN segment as the drone's uXRCE bridge") sugere qualquer host na mesma rede — ambiguo pra producao. Em solo, depende do link de telemetria estar de pe justamente durante uma falha. A bordo, precisa de orcamento de CPU/RAM/disco explicito (nao documentado hoje). | **Alto** | Decisao de arquitetura pendente — recomendo log local a bordo como fonte de verdade (gap #15), replicacao best-effort pro InfluxDB (solo/nuvem) quando o link permitir. |
| 17 | **Grafana nunca foi de fato implantado**, apesar do proprio codigo dizer que existe pra isso: `diagnostics_logger_node.py` linha 2 — `"""InfluxDB v2 logger for /diagnostics_agg — persists drone diagnostic data for Grafana."""`. Nenhum dos dois `docker-compose.yml` sobe um servico Grafana. Hoje, visualizar o historico exige `influx query` manual. | **Medio** | Adicionar servico `grafana` a `ulg_analysis/docker-compose.yml`, com datasource InfluxDB pre-configurado e um dashboard base (20 checks + campos numericos). |
| 18 | **`px4_msgs` precisa bater EXATAMENTE com o firmware do PX4 real**, ou os checks mais criticos falham silenciosamente. Ja documentado como "known issue" com bags SITL antigos (CDR mismatch em `estimator_status`/`vehicle_local_position`/`failsafe_flags`). O simulador tem disciplina forte de pinagem de submodulos (`docs/SUBMODULES.md`) — **nao existe equivalente** para o workspace `~/ros2_humble` do `drone_diagnostics` rodando contra hardware real. Atualizar o firmware do drone sem re-buildar `px4_msgs` na mesma versao faz os checks mais importantes (posicao, EKF, failsafe) falharem silenciosamente. | **Critico** | O processo de release de firmware do drone deve re-buildar/republicar `px4_msgs` pro workspace de diagnostics na mesma versao, com um check de compatibilidade automatico antes de confiar nos dados. |
| 19 | **Thresholds de vibracao/EKF nunca foram calibrados contra uma aeronave real**. Valores default (`imu_accel_vib_warn_ms2=0.35`, `vibration_warn_m2s4=4.0`, etc.) sao chutes razoaveis, mas cada airframe tem assinatura de vibracao/ruido propria. | **Medio** | Fazer 2-3 voos reais "limpos" e usar a distribuicao real de `accel_var_max`/`accel_vibration_metric_ms2` pra recalibrar os thresholds antes de habilitar alertas automaticos (gap #6) em producao. |
| 20 | **Sincronizacao de tempo real precisa ser verificada em campo**: em SITL, `PX4: Time Sync` fica sempre ERROR por causa do clock simulado (ruido conhecido). Em hardware real isso NAO deveria acontecer — se ficar em ERROR num voo real, e sinal genuino de companion computer sem NTP/chrony sincronizado (ou GPS/PPS instavel), nao ruido. **Nao desabilitar** esse check em producao (ao contrario da recomendacao pra SITL no item 8 do plano de remediacao). | **Medio** | Documentar essa diferenca de comportamento esperado SITL-vs-real, pra ninguem desabilitar o check em producao achando que e "ruido de sempre". |

### Prioridade pra ir a campo com hardware real

1. Gap #15 (buffer sem limite / so em RAM) — risco de derrubar o companion
   computer em voo. **Bloqueante.**
2. Gap #18 (pin de `px4_msgs`) — sem isso, os dados mais criticos podem estar
   silenciosamente corrompidos. **Bloqueante.**
3. Gap #14 (fix da tag `level`) — trivial, desbloqueia alertas/dashboards.
4. Gaps #1/#2/#4 do plano principal (restart policy, senha, alertas) — antes
   de qualquer voo "oficial" fora do laboratorio.
5. Gap #16 (decidir topologia: log local a bordo como fonte de verdade) —
   arquitetural, condiciona como o #15 deve ser implementado.
6. Gaps #17, #19, #20 — importantes mas nao bloqueantes pro primeiro voo real
   instrumentado.
