.. _Mjx:

================
MuJoCo XLA (MJX)
================

.. toctree::
    :hidden:

    API <mjx_api.rst>

バージョン3.0.0以降、MuJoCoには `mjx <https://github.com/google-deepmind/mujoco/tree/main/mjx>`__ ディレクトリ配下にMuJoCo XLA（MJX）が含まれています。MJXにより、MuJoCoを `JAX <https://github.com/jax-ml/jax#readme>`__ フレームワークを介して `XLA <https://www.tensorflow.org/xla>`__ コンパイラがサポートする計算ハードウェア上で実行できます。MJXは `JAXがサポートするすべてのプラットフォーム <https://jax.readthedocs.io/en/latest/installation.html#supported-platforms>`__ で動作します：Nvidia および AMD GPU、Apple Silicon、 `Google Cloud TPU <https://cloud.google.com/tpu>`__ 。

MJX APIはMuJoCo APIの主要なシミュレーション関数と一貫性がありますが、現在一部の機能が不足しています。 :ref:`APIドキュメント <Mainsimulation>` は両方のライブラリに適用されますが、MJXでサポートされていない機能については以下の :ref:`注記 <MjxFeatureParity>` で示します。

MJXは `PyPI <https://pypi.org/project/mujoco-mjx>`__ 上で ``mujoco-mjx`` という独立したパッケージとして配布されています。モデルのコンパイルと可視化のために主要な ``mujoco`` パッケージに依存していますが、MJXはMuJoCoと同じアルゴリズムを使用するMuJoCoの再実装です。ただし、JAXを適切に活用するため、MJXは意図的にMuJoCo APIといくつかの点で異なっています。詳細は以下を参照してください。

MJXはGoogleの `Brax <https://github.com/google/brax>`__ 物理・強化学習ライブラリにおける `generalized physics pipeline <https://github.com/google/brax/tree/main/brax/generalized>`__ の後継です。MJXは、MuJoCoとBraxの両方のコア開発者によって構築され、今後も両者（Braxはその強化学習アルゴリズムと含まれる環境のため、MJXはその物理アルゴリズムのため）を一緒にサポートし続けます。Braxの将来のバージョンは ``mujoco-mjx`` パッケージに依存し、Braxの既存の `generalized pipeline <https://github.com/google/brax/tree/main/brax/generalized>`__ は非推奨となります。この変更はBraxユーザーにとってほぼ透過的です。

.. _MjxNotebook:

チュートリアルノートブック
==========================

以下のIPythonノートブックは、MJXを強化学習と組み合わせて使用し、ヒューマノイドおよび四足ロボットの歩行を訓練する方法を示します： |colab| 。

.. |colab| image:: https://colab.research.google.com/assets/colab-badge.png
           :target: https://colab.research.google.com/github/nakamoto-masa/mujoco-docs-ja/blob/main/docs/mjx/tutorial.ipynb

.. _MjxInstallation:

インストール
============

このパッケージをインストールする推奨方法は `PyPI <https://pypi.org/project/mujoco-mjx/>`__ 経由です：

.. code-block:: shell

   pip install mujoco-mjx

MuJoCoライブラリのコピーはこのパッケージの依存関係の一部として提供されており、別途ダウンロードやインストールは **不要** です。

.. _MjxUsage:

基本的な使い方
==============

インストール後、 ``from mujoco import mjx`` でパッケージをインポートできます。構造体、関数、列挙型は、トップレベルの ``mjx`` モジュールから直接利用できます。

.. _MjxStructs:

構造体
------

アクセラレータデバイス上でMJX関数を実行する前に、 ``mjx.put_model`` および ``mjx.put_data`` 関数を介して構造体をデバイスにコピーする必要があります。 :ref:`mjModel` をデバイスに配置すると ``mjx.Model`` が得られます。 :ref:`mjData` をデバイスに配置すると ``mjx.Data`` が得られます：

.. code-block:: python

   model = mujoco.MjModel.from_xml_string("...")
   data = mujoco.MjData(model)
   mjx_model = mjx.put_model(model)
   mjx_data = mjx.put_data(model, data)

これらのMJX版はMuJoCo版に対応していますが、いくつかの重要な違いがあります：

#. ``mjx.Model`` と ``mjx.Data`` にはデバイスにコピーされたJAX配列が含まれます。
#. MJXで :ref:`サポートされていない <mjxFeatureParity>` 機能については、 ``mjx.Model`` と ``mjx.Data`` から一部のフィールドが欠落しています。
#. ``mjx.Model`` と ``mjx.Data`` のJAX配列はバッチ次元の追加をサポートします。バッチ次元は、領域ランダム化（ ``mjx.Model`` の場合）や強化学習のための高スループットシミュレーション（ ``mjx.Data`` の場合）を表現する自然な方法です。
#. ``mjx.Model`` と ``mjx.Data`` のNumpy配列は、JITコンパイルの出力を制御する構造フィールドです。これらの配列を変更すると、JAXはMJX関数を再コンパイルします。例として、 ``jnt_limited`` は :ref:`mjModel` から参照で渡されるnumpy配列で、ジョイント制限制約を適用するかどうかを決定します。 ``jnt_limited`` が変更されると、JAXはMJX関数を再コンパイルします。一方、 ``jnt_range`` は実行時に変更できるJAX配列であり、 ``jnt_limited`` フィールドで指定された制限のあるジョイントにのみ適用されます。


``mjx.Model`` も ``mjx.Data`` も手動で構築することを意図していません。 ``mjx.Data`` は、MuJoCoの :ref:`mj_makeData` 関数に対応する ``mjx.make_data`` を呼び出すことで作成できます：

.. code-block:: python

   model = mujoco.MjModel.from_xml_string("...")
   mjx_model = mjx.put_model(model)
   mjx_data = mjx.make_data(model)

``mjx.make_data`` の使用は、 ``vmap`` 内でバッチ化された ``mjx.Data`` 構造を構築する際に望ましい場合があります。

.. _MjxFunctions:

関数
----

MuJoCo関数は同じ名前のMJX関数として公開されていますが、 `PEP 8 <https://peps.python.org/pep-0008/>`__ に準拠した名前に従っています。 :ref:`主要なシミュレーション <Mainsimulation>` のほとんどと、順動力学シミュレーションの :ref:`サブコンポーネント <Subcomponents>` の一部が、トップレベルの ``mjx`` モジュールから利用できます。

MJX関数はデフォルトでは `JITコンパイル <https://jax.readthedocs.io/en/latest/jax-101/02-jitting.html>`__ されません。ユーザーがMJX関数をJITコンパイルするか、MJX関数を参照する独自の関数をJITコンパイルすることができます。以下の :ref:`最小限の例 <MjxExample>` を参照してください。

.. _MjxEnums:

列挙型と定数
------------

MJX列挙型は ``mjx.EnumType.ENUM_VALUE`` として利用できます。例えば ``mjx.JointType.FREE`` です。サポートされていないMJX機能の列挙型はMJX列挙型宣言から省略されています。MJXは定数を宣言せず、MuJoCo定数を直接参照します。

.. _MjxExample:

最小限の例
----------

.. code-block:: python

   # 100種類の異なる速度でボールを投げる

   import jax
   import mujoco
   from mujoco import mjx

   XML=r"""
   <mujoco>
     <worldbody>
       <body>
         <freejoint/>
         <geom size=".15" mass="1" type="sphere"/>
       </body>
     </worldbody>
   </mujoco>
   """

   model = mujoco.MjModel.from_xml_string(XML)
   mjx_model = mjx.put_model(model)

   @jax.vmap
   def batched_step(vel):
     mjx_data = mjx.make_data(mjx_model)
     qvel = mjx_data.qvel.at[0].set(vel)
     mjx_data = mjx_data.replace(qvel=qvel)
     pos = mjx.step(mjx_model, mjx_data).qpos[0]
     return pos

   vel = jax.numpy.arange(0.0, 1.0, 0.01)
   pos = jax.jit(batched_step)(vel)
   print(pos)

.. _MjxCli:

便利なコマンドラインスクリプト
------------------------------

``mujoco-mjx`` パッケージには2つのコマンドラインスクリプトが用意されています：

.. code-block:: shell

   mjx-testspeed --mjcf=/PATH/TO/MJCF/ --base_path=.

このコマンドはMJCFファイルへのパスとオプションの引数（詳細は ``--help`` を使用）を受け取り、パフォーマンスチューニングに役立つメトリクスを計算します。このコマンドは、他の情報とともに、合計シミュレーション時間、合計ステップ/秒、合計リアルタイム係数（ここで合計とはすべての利用可能なデバイス全体での値）を出力します。

.. code-block:: shell

   mjx-viewer --help

このコマンドは、simulateビューアーでMJXモデルを起動し、モデルを可視化して操作できます。これはMJX物理（C MuJoCoではなく）を使用してシミュレーションをステップ実行するため、例えばソルバーパラメータのデバッグに役立ちます。

.. _MjxFeatureParity:

機能パリティ
============

MJXは、いくつかの例外を除いて、MuJoCoの主要なシミュレーション機能のほとんどをサポートしています。サポートされていない機能を参照するフィールド値を持つ :ref:`mjModel` をデバイスにコピーするよう求められると、MJXは例外を発生させます。

以下の機能はMJXで **完全にサポート** されています：

.. list-table::
   :width: 90%
   :align: left
   :widths: 2 5
   :header-rows: 1

   * - カテゴリ
     - 機能
   * - ダイナミクス
     - :ref:`Forward <mj_forward>` 、 :ref:`Inverse <mj_inverse>`
   * - :ref:`ジョイント <mjtJoint>`
     - ``FREE`` 、 ``BALL`` 、 ``SLIDE`` 、 ``HINGE``
   * - :ref:`伝達 <mjtTrn>`
     - ``JOINT`` 、 ``JOINTINPARENT`` 、 ``SITE`` 、 ``TENDON``
   * - :ref:`アクチュエータダイナミクス <mjtDyn>`
     - ``NONE`` 、 ``INTEGRATOR`` 、 ``FILTER`` 、 ``FILTEREXACT`` 、 ``MUSCLE``
   * - :ref:`アクチュエータゲイン <mjtGain>`
     - ``FIXED`` 、 ``AFFINE`` 、 ``MUSCLE``
   * - :ref:`アクチュエータバイアス <mjtBias>`
     - ``NONE`` 、 ``AFFINE`` 、 ``MUSCLE``
   * - :ref:`テンドン巻き付き <mjtWrap>`
     - ``JOINT`` 、 ``SITE`` 、 ``PULLEY`` 、 ``SPHERE`` 、 ``CYLINDER``
   * - :ref:`ジオム <mjtGeom>`
     - ``PLANE`` 、 ``HFIELD`` 、 ``SPHERE`` 、 ``CAPSULE`` 、 ``BOX`` 、 ``MESH`` は完全に実装されています。 ``ELLIPSOID`` と
       ``CYLINDER`` は実装されていますが、他のプリミティブとのみ衝突します。 ``BOX`` はメッシュとして実装されています。
   * - :ref:`制約 <mjtConstraint>`
     - ``EQUALITY`` 、 ``LIMIT_JOINT`` 、 ``CONTACT_FRICTIONLESS`` 、 ``CONTACT_PYRAMIDAL`` 、 ``CONTACT_ELLIPTIC`` 、
       ``FRICTION_DOF`` 、 ``FRICTION_TENDON``
   * - :ref:`等式制約 <mjtEq>`
     - ``CONNECT`` 、 ``WELD`` 、 ``JOINT`` 、 ``TENDON``
   * - :ref:`積分器 <mjtIntegrator>`
     - ``EULER`` 、 ``RK4`` 、 ``IMPLICITFAST`` （ ``IMPLICITFAST`` は :doc:`流体抗力 <computation/fluid>` ではサポートされていません）
   * - :ref:`錐 <mjtCone>`
     - ``PYRAMIDAL`` 、 ``ELLIPTIC``
   * - :ref:`Condim <coContact>`
     - 1、3、4、6（1は ``ELLIPTIC`` ではサポートされていません）
   * - :ref:`ソルバー <mjtSolver>`
     - ``CG`` 、 ``NEWTON``
   * - 流体モデル
     - :ref:`flInertia`
   * - :ref:`テンドン <tendon>`
     - :ref:`Fixed <tendon-fixed>` 、 :ref:`Spatial <tendon-spatial>`
   * - :ref:`センサー <mjtSensor>`
     - ``MAGNETOMETER`` 、 ``CAMPROJECTION`` 、 ``RANGEFINDER`` 、 ``JOINTPOS`` 、 ``TENDONPOS`` 、 ``ACTUATORPOS`` 、 ``BALLQUAT`` 、
       ``FRAMEPOS`` 、 ``FRAMEXAXIS`` 、 ``FRAMEYAXIS`` 、 ``FRAMEZAXIS`` 、 ``FRAMEQUAT`` 、 ``SUBTREECOM`` 、 ``CLOCK`` 、
       ``VELOCIMETER`` 、 ``GYRO`` 、 ``JOINTVEL`` 、 ``TENDONVEL`` 、 ``ACTUATORVEL`` 、 ``BALLANGVEL`` 、 ``FRAMELINVEL`` 、
       ``FRAMEANGVEL`` 、 ``SUBTREELINVEL`` 、 ``SUBTREEANGMOM`` 、 ``TOUCH`` 、 ``CONTACT`` 、 ``ACCELEROMETER`` 、 ``FORCE`` 、
       ``TORQUE`` 、 ``ACTUATORFRC`` 、 ``JOINTACTFRC`` 、 ``TENDONACTFRC`` 、 ``FRAMELINACC`` 、 ``FRAMEANGACC``
       （ ``CONTACT`` ： ``none-none`` 、 ``geom-geom`` のマッチング；reduction ``mindist`` 、 ``maxforce`` ；data ``all`` ）
   * - ライト
     - ライトの位置と方向

以下の機能は **開発中** で、近日公開予定です：

.. list-table::
   :width: 90%
   :align: left
   :widths: 2 5
   :header-rows: 1

   * - カテゴリ
     - 機能
   * - :ref:`ジオム <mjtGeom>`
     - ``SDF`` 。 （ ``SPHERE`` 、 ``BOX`` 、 ``MESH`` 、 ``HFIELD`` ）と ``CYLINDER`` 間の衝突。 （ ``BOX`` 、 ``MESH`` 、 ``HFIELD`` ）と ``ELLIPSOID`` 間の衝突。
   * - :ref:`積分器 <mjtIntegrator>`
     - ``IMPLICIT``
   * - 流体モデル
     - :ref:`flEllipsoid`
   * - :ref:`センサー <mjtSensor>`
     - ``PLUGIN`` 、 ``USER`` を除くすべて

以下の機能は **サポートされていません** ：

.. list-table::
   :width: 90%
   :align: left
   :widths: 2 5
   :header-rows: 1

   * - カテゴリ
     - 機能
   * - :ref:`margin<body-geom-margin>` と :ref:`gap<body-geom-gap>`
     - ``Mesh`` :ref:`ジオム <mjtGeom>` との衝突では未実装。
   * - :ref:`伝達 <mjtTrn>`
     - ``SLIDERCRANK`` 、 ``BODY``
   * - :ref:`アクチュエータダイナミクス <mjtDyn>`
     - ``USER``
   * - :ref:`アクチュエータゲイン <mjtGain>`
     - ``USER``
   * - :ref:`アクチュエータバイアス <mjtBias>`
     - ``USER``
   * - :ref:`ソルバー <mjtSolver>`
     - ``PGS``
   * - :ref:`センサー <mjtSensor>`
     - ``PLUGIN`` 、 ``USER``
   * - フレックス
     - すべて

.. _MjxSharpBits:

🔪 MJX - The Sharp Bits 🔪
===========================

GPUとTPUには、MJXが影響を受ける独自のパフォーマンストレードオフがあります。MJXは、 `SIMDハードウェア <https://en.wikipedia.org/wiki/Single_instruction,_multiple_data>`__ 上で効率的にベクトル化できるアルゴリズムを使用して、並列で同一の物理シーンの大きなバッチをシミュレートすることに特化しています。この特化は、大量のデータスループットを必要とする `強化学習 <https://en.wikipedia.org/wiki/Reinforcement_learning>`__ などの機械学習ワークロードに有用です。

MJXが不向きなワークフローがいくつかあります：

単一シーンのシミュレーション
  単一シーン（ :ref:`mjData` の1インスタンス）をシミュレートする場合、MJXはCPU向けに慎重に最適化されたMuJoCoよりも **10倍** 遅くなる可能性があります。MJXは数千または数万のシーンを並列でシミュレートする場合に最適に動作します。

大きなメッシュ間の衝突
  MJXは凸メッシュジオメトリ間の衝突をサポートしています。ただし、MJXの凸衝突アルゴリズムはMuJoCoとは異なる実装です。MJXは、 `Separating Axis Test <https://ubm-twvideo01.s3.amazonaws.com/o1/vault/gdc2013/slides/822403Gregorius_Dirk_TheSeparatingAxisTest.pdf>`__（SAT）のブランチレス版を使用して、ジオメトリが凸メッシュと衝突しているかどうかを判定します。一方、MuJoCoはMPRまたはGJK/EPAを使用します。詳細は :ref:`衝突検出<coChecking>` を参照してください。SATは小さなメッシュではうまく機能しますが、大きなメッシュでは実行時間とメモリの両方で問題が発生します。

  凸メッシュとプリミティブ間の衝突では、メッシュの凸分解は合理的なパフォーマンスのために **約200頂点以下** にする必要があります。凸-凸衝突では、凸メッシュは **約32頂点未満** にする必要があります。MuJoCoコンパイラの :ref:`maxhullvert<asset-mesh-maxhullvert>` を使用して、目的の凸メッシュプロパティを達成することをお勧めします。慎重なチューニングにより、MJXはメッシュ衝突のあるシーンをシミュレートできます。例としてMJXの `shadow hand <https://github.com/google-deepmind/mujoco/tree/main/mjx/mujoco/mjx/test_data/shadow_hand>`__ 設定を参照してください。メッシュ衝突検出の高速化は、MJXの開発中の分野です。

多数の接触を持つ大規模で複雑なシーン
  アクセラレータは `分岐コード <https://aschrein.github.io/jekyll/update/2019/06/13/whatsup-with-my-branches-on-gpu.html#tldr>`__ に対して低いパフォーマンスを示します。分岐は、シーン内の多数のボディ間の潜在的な衝突を識別する際のブロードフェーズ衝突検出で使用されます。MJXにはシンプルなブランチレスブロードフェーズアルゴリズムが付属していますが（パフォーマンスチューニングを参照）、MuJoCoのものほど強力ではありません。

  これがシミュレーションにどのように影響するかを見るため、1から10まで変化するヒューマノイドボディの数を増やした物理シーンを考えてみましょう。Apple M3 MaxとAMD 3995WX 64コアCPU上でMuJoCoを使用してこのシーンをシミュレートし、 ``2 x numcore`` スレッドを使用して :ref:`testspeed<saTestspeed>` で時間を測定します。Nvidia A100 GPU上でバッチサイズ8192を使用したMJXシミュレーション、および8チップ `v5 TPU <https://cloud.google.com/blog/products/compute/announcing-cloud-tpu-v5e-and-a3-gpus-in-ga>`__ マシン上でバッチサイズ16384を使用したMJXシミュレーションの時間を測定します。縦軸スケールは対数です。

  .. figure:: images/mjx/SPS.svg
     :width: 95%
     :align: center

  単一ヒューマノイド（最左のデータポイント）の4つの測定アーキテクチャの値は、それぞれ **650K** 、 **1.8M** 、 **950K** 、 **2.7M** ステップ/秒です。ヒューマノイドの数を増やすと（シーン内の潜在的な接触数が増加）、MJXのスループットはMuJoCoよりも急速に低下することに注意してください。

.. _MjxPerformance:

パフォーマンスチューニング
==========================

MJXが良好に動作するためには、デフォルトのMuJoCo値からいくつかの設定パラメータを調整する必要があります：

:ref:`option/iterations<option-iterations>` と :ref:`option/ls_iterations<option-ls_iterations>`
  :ref:`iterations<option-iterations>` と :ref:`ls_iterations<option-ls_iterations>` 属性（それぞれソルバーとラインサーチの反復を制御）は、シミュレーションが安定したままになるちょうど低いレベルまで下げる必要があります。正確なソルバー力は、シム・トゥ・リアルのために物理にノイズを追加するために領域ランダム化がしばしば使用される強化学習においてはそれほど重要ではありません。 ``NEWTON`` :ref:`ソルバー <mjtSolver>` は非常に少ない（多くの場合わずか1回の）ソルバー反復で優れた収束を実現し、GPU上で良好に動作します。 ``CG`` は現在TPUにとってより良い選択です。

:ref:`contact/pair<contact-pair>`
  MJXが各ステップ中に考慮する必要がある接触の数を減らすために、衝突検出のためにジオムを明示的にマークすることを検討してください。有効な接触の明示的なリストのみを有効にすることは、MJXのシミュレーションパフォーマンスに劇的な効果をもたらす可能性があります。これをうまく行うには、多くの場合タスクの理解が必要です。例えば、 `OpenAI Gym Humanoid <https://github.com/openai/gym/blob/master/gym/envs/mujoco/humanoid_v4.py>`__ タスクは、ヒューマノイドが倒れ始めるとリセットされるため、床との完全な接触は必要ありません。

:ref:`maxhullvert<asset-mesh-maxhullvert>`
   より良い凸メッシュ衝突パフォーマンスのために、 :ref:`maxhullvert<asset-mesh-maxhullvert>` を `64` 以下に設定してください。

:ref:`option/flag/eulerdamp<option-flag-eulerdamp>`
  ``eulerdamp`` を無効にすると、パフォーマンスが向上し、多くの場合安定性のために必要ありません。このフラグのセマンティクスに関する詳細は :ref:`数値積分<geIntegration>` セクションを参照してください。

:ref:`option/jacobian<option-jacobian>`
  「dense」または「sparse」を明示的に設定すると、デバイスに応じてシミュレーションが高速化される場合があります。最新のTPUは疎行列を高速に操作するための専用ハードウェアを持っていますが、GPUは密行列がデバイスに収まる限り、密行列の方が高速である傾向があります。そのため、MJXのデフォルト「auto」設定の動作は、 ``nv >= 60`` （60以上の自由度）の場合、またはMJXがデフォルトバックエンドとしてTPUを検出した場合はsparseで、それ以外はdenseです。TPUの場合、Newtonソルバーで「sparse」を使用すると、シミュレーションが2倍から3倍高速化されます。GPUの場合、密行列がデバイスに収まる限り、「dense」を選択すると10％から20％程度の控えめな速度向上が得られる場合があります。

ブロードフェーズ
  MuJoCoはブロードフェーズのカリングをすぐに処理しますが、MJXには追加のパラメータが必要です。ブロードフェーズの近似版には、実験的なカスタム数値パラメータ ``max_contact_points`` と ``max_geom_pairs`` を使用してください。 ``max_contact_points`` は各condimタイプのソルバーに送信される接触点の数に上限を設けます。 ``max_geom_pairs`` は各ジオムタイプペアのそれぞれの衝突関数に送信されるジオムペアの総数に上限を設けます。例として、 `shadow hand <https://github.com/google-deepmind/mujoco/tree/main/mjx/mujoco/mjx/test_data/shadow_hand>`__ 環境がこれらのパラメータを利用しています。

GPUパフォーマンス
-----------------

以下の環境変数を設定する必要があります：

``XLA_FLAGS=--xla_gpu_triton_gemm_any=true``
  これにより、サポートするすべてのGEMMに対してTritonベースのGEMM（matmul）エミッタが有効になります。これによりNVIDIA GPUで30％の速度向上が得られます。複数のGPUがある場合は、 `GPU間の通信 <https://jax.readthedocs.io/en/latest/gpu_performance_tips.html>`__ に関連するフラグを有効にすることでも恩恵を受ける可能性があります。
