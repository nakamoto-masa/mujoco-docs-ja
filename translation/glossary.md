# MuJoCo ドキュメント翻訳 用語集

## 翻訳ガイドライン

- MuJoCo固有の用語（API名、構造体名、フォーマット名等）は英語のまま残す
- 物理・数学の一般用語は日本語に翻訳する
- カタカナ表記が定着している用語はカタカナを使用する
- 分かりにくい場合は適宜原語を併記

## 翻訳しない用語（英語のまま）

### C構造体名・API関数名

`mj` または `mjs` で始まる識別子（構造体名、関数名）は英語のまま残す。

例：mjData, mjModel, mjsBody, mj_step, mjs_addGeom

### 略語

| 英語 | 備考 |
|------|------|
| DOF | Degrees of Freedom。本文では「自由度（DOF）」と併記 |
| IMU | Inertial Measurement Unit。初出時に「慣性計測ユニット（IMU）」と併記 |
| LCP | Linear Complementarity Problem。初出時に日本語を併記 |
| NCP | Nonlinear Complementarity Problem。初出時に日本語を併記 |

## 翻訳する用語

### 物理・数学

| 英語 | 日本語 | 備考 |
|------|--------|------|
| axis-aligned bounding box | 軸並行バウンディングボックス | AABB |
| Cartesian coordinates | 直交座標 | |
| centripetal force | 遠心力 | |
| Cholesky factorization | コレスキー分解 | |
| conjugate gradient | 共役勾配 | |
| constraint island | 制約アイランド | |
| convex optimization | 凸最適化 | |
| Coriolis force | コリオリ力 | |
| critical damping | 臨界減衰 | |
| deformation | 変形 | 制約の文脈 |
| degrees of freedom | 自由度 | |
| derivative | 導関数 | |
| dual problem | 双対問題 | |
| equivalent-inertia box | 等価慣性ボックス | |
| Euler rotation | オイラー回転 | |
| explicit Euler | 陽的オイラー | |
| finite differences | 有限差分 | |
| forward dynamics | 順動力学 | |
| forward kinematics | 順運動学 | |
| generalized coordinates | 一般化座標 | |
| gyroscopic force | ジャイロスコピック力 | |
| Hessian | ヘッセ行列 | |
| implicit Euler | 陰的オイラー | |
| inertia | 慣性 | |
| kinetic energy | 運動エネルギー | |
| inverse dynamics | 逆動力学 | |
| inverse kinematics | 逆運動学 | |
| joint coordinates | 関節座標 | |
| Lagrange multiplier | ラグランジュ乗数 | |
| line-search | 直線探索 | |
| linear complementarity problem | 線形相補性問題 | |
| Lyapunov exponent | リアプノフ指数 | |
| Newton's method | ニュートン法 | |
| numerical integration | 数値積分 | |
| perturbation | 摂動 | |
| physics engine | 物理エンジン | |
| potential energy | ポテンシャルエネルギー | |
| primal problem | 主問題 | |
| Projected Gauss-Seidel | 射影ガウス・ザイデル | PGS |
| quaternion | クォータニオン | |
| raycasting | レイキャスティング | |
| reduced primal problem | 縮約主問題 | |
| regularizer | 正則化項 | |
| rigid body | 剛体 | |
| Runge-Kutta | ルンゲ・クッタ | |
| semi-implicit Euler | 半陰的オイラー | |
| state | 状態 | シミュレーション状態 |

### 接触・摩擦・制約

| 英語 | 日本語 | 備考 |
|------|--------|------|
| bounding volume hierarchy | バウンディングボリューム階層 | BVH |
| broad-phase collision detection | ブロードフェーズ衝突検出 | |
| collider | コライダー | |
| collision detection | 衝突検出 | |
| connect constraint | 接続制約 | |
| constraint | 制約 | |
| contact dynamics | 接触動力学 | |
| contact exclude | 接触除外 | |
| contact normal | 接触法線 | |
| contact pair | 接触ペア | |
| convex decomposition | 凸分解 | |
| dry friction | 乾燥摩擦 | |
| elliptic cone | 楕円錐 | 摩擦円錐の形状 |
| equality constraint | 等式制約 | |
| friction cone | 摩擦円錐 | |
| friction loss | 摩擦損失 | |
| inequality constraint | 不等式制約 | |
| loop joint | ループジョイント | キネマティックツリー外のジョイント |
| narrow-phase collision detection | ナローフェーズ衝突検出 | |
| pyramidal cone | 角錐 | 摩擦円錐の形状 |
| residual | 残差 | 制約の文脈 |
| rolling friction | 転がり摩擦 | |
| slip | 滑り | |
| softness | 柔軟性 | |
| torsional friction | ねじり摩擦 | |
| weld constraint | 溶接制約 | |

### モデル要素

| 英語 | 日本語 | 備考 |
|------|--------|------|
| ball joint | ボールジョイント | |
| body | ボディ | MuJoCo用語としてカタカナ |
| flex | フレックス | MuJoCo用語としてカタカナ |
| floating body | 浮遊ボディ | |
| frame | フレーム | MuJoCo用語としてカタカナ |
| free joint | フリージョイント | |
| geom | ジオム | MuJoCo用語としてカタカナ |
| hinge joint | ヒンジジョイント | |
| joint | ジョイント | MuJoCo用語としてカタカナ |
| keyframe | キーフレーム | |
| kinematic tree | キネマティックツリー | |
| meta-element | メタ要素 | |
| section | セクション | |
| site | サイト | MuJoCo用語としてカタカナ |
| slide joint | スライドジョイント | |
| tendon | テンドン | MuJoCo用語としてカタカナ |

### 形状・外観

| 英語 | 日本語 | 備考 |
|------|--------|------|
| capsule | カプセル | |
| convex hull | 凸包 | |
| cylinder | 円筒 | |
| ellipsoid | 楕円体 | |
| height field | ハイトフィールド | |
| material | マテリアル | |
| skin | スキン | |

### 変形可能オブジェクト

| 英語 | 日本語 | 備考 |
|------|--------|------|
| bind pose | バインドポーズ | |
| bone | ボーン | |
| edge | エッジ | |
| elasticity | 弾性 | |
| element | 要素 | |
| Poisson's ratio | ポアソン比 | |
| Rayleigh damping | レイリー減衰 | |
| strain | ひずみ | |
| stress | 応力 | |
| vertex | 頂点 | |
| Young's modulus | ヤング弾性率 | ヤング率とも |

### アクチュエーション・力学特性

| 英語 | 日本語 | 備考 |
|------|--------|------|
| activation dynamics | 活性化ダイナミクス | |
| armature inertia | アーマチュア慣性 | |
| damping | 減衰 | |
| force generation | 力の生成 | |
| moment arm | モーメントアーム | |
| spring-damper | バネ・ダンパー | |
| stiffness | 剛性 | |
| transmission | 伝達 | アクチュエータの文脈 |
| via-point | 経由点 | |
| wrapping | 巻き付き | |

### シミュレーション・可視化

| 英語 | 日本語 | 備考 |
|------|--------|------|
| deterministic | 決定論的 | |
| divergence | 発散 | |
| impedance | インピーダンス | 制約の文脈 |
| integrator | 積分器 | |
| multi-threading | マルチスレッド | |
| reproducibility | 再現性 | |
| sleeping island | スリーピングアイランド | |
| skybox | スカイボックス | |
| solver | ソルバー | |
| time step | タイムステップ | |
| timestep | タイムステップ | time step の表記揺れ |
| visualization | 可視化 | |
| warmstart | ウォームスタート | 制約ソルバーの初期化手法 |

### センサー

| 英語 | 日本語 | 備考 |
|------|--------|------|
| accelerometer | 加速度計 | |
| camprojection | カメラ投影 | |
| frustum | 錐台 | |
| gyroscope | ジャイロスコープ | |
| magnetometer | 磁力計 | |
| rangefinder | 測距計 | |
| touch sensor | タッチセンサー | |
| velocimeter | 速度計 | |

### プログラミング・API

| 英語 | 日本語 | 備考 |
|------|--------|------|
| asset | アセット | |
| attachment | アタッチメント | |
| compile | コンパイル | mjSpecからmjModelへの変換 |
| deep copy | ディープコピー | |
| default class | デフォルトクラス | |
| in-place | インプレース | |
| opaque type | 不透明型 | C++内部型をCに公開 |
| parse | 解析 | XMLからmjSpecへの変換 |
| procedural | プロシージャル | |
| recompilation | 再コンパイル | |
| shallow copy | シャローコピー | |
| subtree | サブツリー | |
