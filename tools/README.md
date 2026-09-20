# tools/ — 仿真与可视化工具链

```
tools/
  full_sim.py         全闭环仿真 (NN规划器 + VST + LQR + LADRC + 小车动力学)
  sim_compare.py      仿真 vs 实车对比图
  plot_run.py         CSV → 三图波形 (轨迹/航向/误差+控制量)
  plot_traj_stats.py  CSV → 纯轨迹图 + 三项误差统计标注
  plot_resync.py      VST 重同步开/关对比图
  animate_scope.py    CSV → 小车+示波器动画 (MP4, 1920×1080)
  ctrl_bench.py       控制器性能对比 (设计 wo 模型 vs 轮胎侧滑模型)
  gen_lqr.py          LQR 增益表离线求解 → project/code/core/lqr_gains.h
  tune_lqr.py         LQR 权重 (q_ey,r) 扫描调参
  data/               CSV (实车录制 + 仿真输出)
  out/                图/视频产物
  docs/               控制器推导 (LaTeX)
```

## 数据流

```
full_sim.py ──> data/sim_run*.csv ──┬─> plot_run.py         out/*.png
                                    ├─> plot_traj_stats.py  out/*_traj.png
                                    ├─> animate_scope.py    out/*_scope.mp4
                                    └─> sim_compare.py      out/sim_vs_real*.png
data/run_data.csv (实车录制) ───────┘
```

## 常用命令

```bash
python tools/full_sim.py --mode 2                    # 6 航点自跑 → data/sim_run.csv
python tools/full_sim.py --mode 1 --plot             # 位置阶跃 → data/sim_step.csv
python tools/full_sim.py --c-model nn_model_sym3_d5v15.c  # 切 d5v15
python tools/full_sim.py --v-max 5.0                 # 覆盖部署限速 (只影响仿真)
python tools/full_sim.py --ckpt <path.pt>            # 直接加载训练检查点 (未折叠权重)
python tools/full_sim.py --tire --mu 0.55 --cf 60    # 轮胎侧偏模型
python tools/full_sim.py --yaw-noise 2.0             # 实车等效: 加 IMU 航向噪声
python tools/full_sim.py --no-resync                 # 关 VST 5cm 重同步 (鲁棒性研究)

python tools/ctrl_bench.py                           # 控制器性能对比 → out/ctrl_bench.png
python tools/plot_run.py data/sim_run.csv
python tools/animate_scope.py data/sim_run.csv --speed 2
python tools/sim_compare.py data/run_data.csv data/sim_run.csv out/cmp.png
```

## data/ 文件说明

| 文件 | 说明 |
|---|---|
| `run_data.csv` | **实车录制原始数据** (不可再生, 勿覆盖) |
| `sim_run.csv` | 默认模型 (gpmed_v4) 输出 |
| `sim_run_noresync.csv` | 同模型关闭 VST 重同步 (鲁棒性对照) |
| `sim_step.csv` | MODE 1 位置阶跃 (不经过 NN, 与模型无关) |

## 弯路记录: HOME 绕圈问题定位 (2026-09)

**现象**: 仿真在最后一个航点 (HOME) 首次进场擦边错过 → 掉头绕回 → 二次命中。
实车同期数据只录到穿过 HOME 点为止 (9.55s, v=4.31 m/s), 看不出是否也有此现象。

**调查链路 (前三步都是弯路)**:

1. **怀疑横向控制器 / 车辆模型** → 引入轮胎侧偏力模型 (`TireBicyclePlant`, 滑移角+侧偏刚度+μFz 饱和)
   → 扫 20 组 (μ, C) 全部无法复现实车 eth 统计 (2.6°/8.5°) → **排除**
   → 副产物: 发现 wo=15 一阶惯性 ≈ 轮胎模型 (C≈60) + 舵机 50ms 滞后, 是同一物理的不同集总层

2. **怀疑 VST 重同步** → 关掉后确实复现绕圈 → 但实车 |VST−car| 全程 <4.6cm, 重同步**零触发**
   → 实车定位确认: 重同步是意外兜底, 不是主因。仿真的漂移 (6.6cm) 比实车大, 才让它变成常触发

3. **怀疑噪声标定** → yaw-noise 2° 使 eth 统计精确吻合 (2.5/8.2 vs 实车 2.6/8.5)
   → 提高了仿真逼真度, 但**绕圈依旧** (8 seeds 仍有 ~5/8 错过)

4. **最终定位: NN 规划器** — 旧网络 (Aug-15) 在 3.73m 长腿进场时收敛慢, HOME 首次进场 9~12cm,
   贴着 10cm 撞线圈边缘。换 Sep-8 v4 分布网络 → 首次进场 2.3cm, **8 seeds 全部首次进圈**

**2×2 分解 (容量 vs 训练侧)**:

| 配置 | 参数 | 训练来源 | HOME 首次进场 | 首次进圈 |
|---|---|---|---|---|
| A 旧部署 C (`nn_model_a5b3l4.c`) | 3762 | 旧 teacher + 旧分布 | 10.6 cm | 3/8 |
| B Sep-8 GP-Small | 3762 | **v4 分布** | 2.4 cm | 8/8 |
| C Sep-8 GP-Medium | 8990 | **v4 分布** | 2.3 cm | 8/8 |

- **容量贡献 ≈ 0** (B→C: 0.1cm) — 项目自带基准 `bench_loops.py` 独立确认 (中网络 1 胜 2 负 3 平)
- **训练侧贡献 8.2cm** (A→B) — v4 半正态截断分布 [0.05, 6.5] 真实覆盖长腿目标
- 例外: `zigzag6` 场景中网络明显更好 (15.0s vs 21.1s, 绕行 4 vs 5) — 连续急转是容量敏感场景
- 注: A→B 同时混了 teacher 重训与分布改动, 无法再细分; B→C 是干净的单变量对比

**弯路产物已清理** (2026-09-12): 旧模型 (a5b3l4) 的仿真数据与图已从 `data/` `out/` 删除,
固件侧旧模型文件也已移除。轮胎模型与噪声标定能力保留在 `full_sim.py`
(`--tire` / `--yaw-noise`) —— 抓地极限工况 + 用 4 状态 A(v) 重设计 LQR 增益表的试验台。

## 控制器性能: 设计模型 vs 轮胎模型 (ctrl_bench.py)

**测试 A — 车道保持阶跃** (5cm→0, ±0.5cm 带, 舵机不饱和, 调节时间 [s]):

| v [m/s] | wo=15 设计 | wo=8 失配 | wo=25 失配 | 轮胎 C=60 | 轮胎 C=200 | 轮胎 C=30 |
|---|---|---|---|---|---|---|
| 1.0 | 0.545 | 0.515 | 0.565 | 0.625 | 0.650 | 0.605 |
| 2.0 | 0.300 | 0.505 | 0.315 | 0.345 | 0.370 | 0.630 |
| 3.0 | 0.220 | 0.380 | 0.230 | 0.490 | 0.270 | 0.660 |
| 5.0 | 0.150 | 0.270 | 0.155 | 0.505 | 0.195 | 1.035 |

- 全程 **零超调、零稳态误差** (Cf=Cr 中性转向 → 稳态增益与设计一致); 所有工况稳定
- wo=25 (真实比设计快) ≈ 设计; wo=8 (慢 2×) 慢 1.6~1.8×
- 轮胎模型随速度恶化, 且**软硬影响极大**: C=200 ≈ 设计, C=30 最差 (v=5 时 1.035s, 7× 于设计)

**机理 — 两个独立失配, 结构失配才是主因**:

1. 带宽失配: 设计恒定 wo=15; 轮胎 yaw 极点 = 2lf²C/(Iz·v) ∝ 1/v
   (C=60: v=5→13.5, v=1→67.5; C=200: v=5→45; C=30: v=5→6.75)
2. **结构失配 (决定性)**: 设计假设 ėy ≈ v·eth (无侧滑); 真实 ėy = vy + v·eth。
   实测 vy/(v·eth) 峰值: **C=200@5m/s = 0.54** (航向主导 → 结构匹配 → 最快) /
   C=60@5 = 2.92 / C=30@5 = 4.23 (侧滑主导 → 失配 → 最慢)。
   C=200 的带宽离设计最远 (45 vs 15) 却表现最好 → **带宽数值不是决定因素**

**测试 B — 全场 6 航点** (ey RMS [cm], 新网络):

| 被控对象 | 0-1 m/s | 1-2 | 2-3 | 3-4 |
|---|---|---|---|---|
| wo=15 设计 | 0.95 | 0.89 | 0.62 | 0.17 |
| wo=8 失配 | 1.05 | **1.30** | 0.99 | 0.34 |
| 轮胎 C=60 | 0.90 | 0.67 | 0.46 | 0.15 |
| 轮胎 C=200 | 0.86 | 0.61 | 0.40 | 0.10 |
| 轮胎 C=30 | 0.95 | 0.78 | 0.56 | 0.24 |

实际任务剖面 (速度多在 1~3 m/s、参考平滑) 下, 各被控对象差异仅 0.09~1.37cm —
失配影响被低速段稀释; 唯一明显劣化是 wo=8 (真实比设计慢)。

**结论**: 控制器在两种模型下均稳定且精度足够 (全场 ey ≤ 1.4cm RMS)。失配代价取决于
**侧滑是否显著** (是否偏离 ėy≈v·eth), 而非带宽数值大小。已知弱点: 无积分项 →
对持续侧向扰动 (坡道/侧风) 存在稳态误差 (未实测)。

## 模型约束 (工厂模式)

`full_sim.py` 不再硬编码模型约束 —— 环境参数**取自所选模型的 `model_desc_t`**,
与固件 `g_model_active` 同一套来源 (工厂模式):

```python
desc = load_model_desc_from_c(CODE/'core'/args.c_model)   # --c-model 选定
# → a_long_max / a_brake_max / a_lat_max / v_max / nn_a_max / nn_a_brake_max / nn_o_max
```

**注意 `model_desc_t` 里混了两种语义**:

| 字段 | 语义 | 来源 |
|---|---|---|
| `a_long_max` / `a_brake_max` / `a_lat_max` / `nn_*` | **训练包络** (模型训练时的环境约束) | 训练 pipeline |
| `v_max` | **部署限制** (实车安全上限, 可手工下调) | 操作者 |

例: `sym3_d5v15` 训练时 `v_max=5.0`, 部署时为了实车安全手工改成 `1.0`。
所以仿真按 `model_desc_t` 跑 = **按部署状态镜像固件**; 要复现"训练包络下的行为"
用 `--v-max 5.0` 在仿真内覆盖 (不动 C 文件, 避免连带改到固件):

```bash
python tools/full_sim.py --c-model nn_model_sym3_d5v15.c --v-max 5.0
#   v_max=1 (部署): 圈时 18.94s   |   v_max=5 (训练包络): 12.54s
```

| 模型 | a_long | a_lat | v_max | 说明 |
|---|---|---|---|---|
| gpmed_v4 | 5.0 | 3.0 | 5.0 | 当前主力 |
| sym3_d5v15 | 3.0 | 3.0 | 1.0 | 训练 5.0 / 部署下限速 |

`--ckpt` 加载权重时, 约束仍取自 `--c-model` (两者独立: 权重来源 vs 环境来源)。
