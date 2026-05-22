## Problema que Queremos Resolver

O drone Hermit opera com ganhos PID definidos manualmente no PX4. Esse processo é lento, depende da experiência do operador e não garante desempenho ótimo — especialmente em voo autônomo com controle de posição. Queremos **substituir o ajuste manual por um pipeline automatizado** que:

1. Identifica o modelo físico do drone a partir de logs de voo reais (ULG)
2. Simula o comportamento do controlador em software
3. Otimiza os 23 ganhos do cascadeamento PID (taxa → atitude → velocidade → posição) usando algoritmos de busca
4. Exporta os ganhos otimizados diretamente no formato `.params` do PX4

O objetivo final é **reduzir o erro de rastreamento de posição e tornar o processo de ajuste repetível e auditável** para qualquer configuração de drone da equipe.

---

## O que foi construído

- **Pipeline de 5 notebooks:** Exploração ULG → Identificação de Sistema → Simulação → Otimização do modelo (GA) → Exportação de ganhos
- **Biblioteca principal** (`pid_optimizer/`): `plant_model.py`, `gains.py`, `pid_simulator.py`, `ga.py`, `params_io.py`
- **Espaço de busca com 23 ganhos PID** distribuídos em 4 loops em cascata (taxa / atitude / velocidade / posição)
- **Suite de testes completa** (5 arquivos de teste, todos passando com pytest)
- **Pipeline de ingestão InfluxDB** para logs de voo automáticos via `ulg_ingestor/`

---

## O que foi alcançado até o momento

- Modelo do drone identificado a partir de dados reais de voo (massa 1,8 kg, kT = 23,8 N/thr², Ixx/Iyy/Izz estimados)
- Simulador PX4 em cascata construído; modo `run_translational` evita divergência de atitude fixando-a a partir do log
- Framework de AG funcional: **melhoria de 38% no fitness** em 5 gerações sobre os parâmetros do modelo
- RMSE no eixo Y caiu de **1,98 m → 0,36 m** via otimização GA do modelo. Entretanto, temos problemas com a simulação e problemas em outros eixos

---

## O que falhou

- **Otimização de ganhos (notebook 05):** todos os melhores candidatos do AG retornam `fitness = 1e6` (penalidade / crash da simulação)
  - Controlador diverge sob ganhos modificados (saturação, windup integral, kick derivativo)
  - `z_vel_D` mutou de 0 → 1,52 → NaN/overflow no simulador
- **Identificação de inércia (notebook 02):** correlação do balanço de torque r ≈ 0,05 — ruído de medição muito alto
  - Izz frequentemente negativo → necessita clamping; coeficientes de arrasto usados como padrão (fase de planagem muito curta)
- **Identificabilidade de massa/kT:** anti-correlacionados na função de fitness — o AG não consegue separá-los de forma única
- **RMSE de posição ainda alto após GA do modelo:** x = 2,28 m (111% do range), y = 7,13 m (929% do range)

---

## O que Precisamos Corrigir e Validar

- [ ] Proteger o otimizador de ganhos contra crashes: limite de kick derivativo, clamp de windup integral por candidato
- [ ] Restringir magnitude de mutação (máx. ±50% do nominal por passo)
- [ ] Rodar o AG por 200+ gerações (atualmente limitado a 5 por velocidade)
- [ ] Melhorar a identificação do modelo com manobras diversas: step de thrust, hover + sweep de pitch, rampa de aceleração
- [ ] Resolver acoplamento massa/kT: calibrar kT em bancada separadamente para que a massa seja entrada fixa do AG
- [ ] Validar a simulação 6DOF em malha fechada contra logs reais (atualmente apenas o modo translacional foi validado)
- [ ] Adicionar diagnósticos de convergência: análise de sensibilidade por parâmetro, gráficos do landscape de fitness
- [ ] Pensar em melhores formas de replicar os calculos da PX4 para com os dados sinteticos se aproximar do real

---

## Métodos Alternativos de Otimização para Explorar

| Método | Pontos Fortes | Esforço | Notas |
|--------|---------------|---------|-------|
| **CMA-ES** | Superior ao AG em espaços contínuos; adapta a covariância | Baixo | `pip install cma`; substituto direto do AG |
| **Otimização Bayesiana** | ~10× menos avaliações via surrogate de Processo Gaussiano | Médio | `optuna` ou `scikit-optimize`; ideal para fitness caro |
| **PX4 Auto-tune** (`MC_AUTOTUNE_ANG_*`) | Sem simulação; varredura chirp no drone real | Baixo | Embutido no PX4; sysid no domínio da frequência → ganhos |
| **Relay Feedback (Åström-Hägglund)** | Clássico; identifica Ku/Pu → Ziegler-Nichols | Baixo | Apenas hardware real; funciona bem para o loop de taxa |
| **LQR / H∞** | Ganhos lineares ótimos a partir de função de transferência identificada | Alto | Requer boa identificação de sistema primeiro; robusto a incertezas |
| **Aprendizado por Reforço (PPO/SAC)** | Lida com não-linearidades; política direta | Muito Alto | Overkill a menos que a fidelidade do simulador seja alta |
