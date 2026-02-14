.. _MJW:

====================
MuJoCo Warp (MJWarp)
====================

.. toctree::
    :hidden:

    API <api.rst>

MuJoCo Warp (MJWarp) は `Warp <https://nvidia.github.io/warp/>`__ で実装されたMuJoCoで、
`NVIDIA <https://nvidia.com>`__ ハードウェアと並列シミュレーションに最適化されています。MJWarpは
`google-deepmind/mujoco_warp <https://github.com/google-deepmind/mujoco_warp>`__ GitHubリポジトリに存在します。

MJWarpは `NVIDIA <https://nvidia.com>`__ と
`Google DeepMind <https://deepmind.google/>`__ の共同開発によって維持されています。

.. _MJW_tutorial:

チュートリアルノートブック
==========================

MJWarpの基本は
`チュートリアル
ノートブック <https://colab.research.google.com/github/google-deepmind/mujoco_warp/blob/main/notebooks/tutorial.ipynb>`__ でカバーされています。

MJWarpを使うべき場面
====================

.. TODO(robotics-simulation): batch renderer

高スループット
--------------

MuJoCoエコシステムはバッチシミュレーションのための複数のオプションを提供しています。

- :ref:`mujoco.rollout <PyRollout>`: CPU上で :ref:`mj_step` をマルチスレッド呼び出しするためのPython API。高スループット
  は高速コアと大きなスレッド数を持つハードウェアで達成できますが、頻繁なホスト<>デバイス転送を必要とするアプリケーション
  （例：CPU上のシミュレーションとGPU上の学習を使用する強化学習）の全体的なパフォーマンスは転送オーバーヘッドによってボトルネックになる可能性があります。
- **mjx.step**: `jax.vmap` と `jax.pmap` はCPU、GPU、またはTPU上でJAXによるマルチスレッドおよびマルチデバイスシミュレーションを可能にします。
- :func:`mujoco_warp.step <mujoco_warp.step>`: NVIDIA GPU上でWarp経由のCUDAを使用したマルチスレッドおよびマルチデバイスシミュレーションのためのPython API。
  MJXのJAX実装と比較して、接触が多いシーンでのスケーリングが改善されています。

.. TODO(robotics-simulation): add link to mjx.step
.. TODO(robotics-simulation): add step/time comparison plot

低レイテンシ
------------

MJWarpはスループット（単位時間当たりの総シミュレーションステップ数）に最適化されていますが、MuJoCoはレイテンシ（1シミュレーションステップの時間）に最適化されています。
同じシミュレーションに対して、MJWarpでのシミュレーションステップはMuJoCoでのステップよりもパフォーマンスが低いことが予想されます。

その結果、MJWarpは強化学習のような大量のサンプルが必要なアプリケーションに適していますが、MuJoCoはオンライン制御（例：モデル予測制御）やインタラクティブなグラフィカルインターフェース（例：シミュレーションベースのテレオペレーション）のようなリアルタイムアプリケーションにより有用です。

複雑なシーン
------------

MJWarpは、多くのジオムや自由度を持つシーンでMJXよりもスケールしますが、MuJoCoほどではありません。60自由度を超えるシーンでは、MJWarpで大幅なパフォーマンス低下が発生する可能性があります。
これらの大規模なシーンのサポートは高優先度であり、進捗はGitHub issueで追跡されています：スパースヤコビアン
`#88 <https://github.com/google-deepmind/mujoco_warp/issues/88>`__ 、ブロックコレスキー分解と解法
`#320 <https://github.com/google-deepmind/mujoco_warp/issues/320>`__ 、制約アイランド
`#886 <https://github.com/google-deepmind/mujoco_warp/issues/886>`__ 、スリーピングアイランド
`#887 <https://github.com/google-deepmind/mujoco_warp/issues/887>`__ 。

.. TODO(robotic-simulation): add graph for ngeom and nv scaling

微分可能性
----------

MJXのダイナミクスAPIはJAXによって自動的に微分可能です。Warp経由でMJWarpでこれをサポートするかどうかを検討しています。この機能が重要な場合は、このissue
`こちら <https://github.com/google-deepmind/mujoco_warp/issues/500>`__ にご意見をお寄せください。

.. TODO(robotics-simulation): Newton multi-physics

.. _MJW_install:

インストール
============

MuJoCo Warpのベータ版はGitHubからインストールされます。MuJoCo Warpのベータ版はすべてのバージョンのMuJoCo、Warp、CUDA、NVIDIAドライバなどをサポートしているわけではないことにご注意ください。

.. code-block:: shell

    git clone https://github.com/google-deepmind/mujoco_warp.git
    cd mujoco_warp
    python3 -m venv env
    source env/bin/activate
    pip install --upgrade pip
    pip install uv
    uv pip install -e .[dev,cuda]

インストールのテスト

.. code-block:: shell

    pytest

.. _MJW_Usage:

基本的な使い方
==============

インストール後、パッケージは ``import mujoco_warp as mjw`` でインポートできます。構造体、関数、列挙型は
トップレベルの :mod:`mjw <mujoco_warp>` モジュールから直接利用できます。

構造体
------
NVIDIA GPU上でMJWarp関数を実行する前に、構造体は
:func:`mjw.put_model <mujoco_warp.put_model>` と :func:`mjw.make_data <mujoco_warp.make_data>` または
:func:`mjw.put_data <mujoco_warp.put_data>` 関数を介してデバイスにコピーする必要があります。デバイス上に :ref:`mjModel` を配置すると
:class:`mjw.Model <mujoco_warp.Model>` が得られます。デバイス上に :ref:`mjData` を配置すると
:class:`mjw.Data <mujoco_warp.Data>` が得られます。

.. code-block:: python

  mjm = mujoco.MjModel.from_xml_string("...")
  mjd = mujoco.MjData(mjm)
  m = mjw.put_model(mjm)
  d = mjw.put_data(mjm, mjd)

これらのMJWarp版は、MuJoCoの対応するものを反映していますが、いくつかの重要な違いがあります：

#. :class:`mjw.Model <mujoco_warp.Model>` と :class:`mjw.Data <mujoco_warp.Data>` は、デバイスにコピーされたWarp配列を含んでいます。
#. :class:`mjw.Model <mujoco_warp.Model>` と :class:`mjw.Data <mujoco_warp.Data>` から、サポートされていない機能のためにいくつかのフィールドが欠けています。

バッチサイズ
------------

MJWarpは並列シミュレーションに最適化されています。シミュレーションのバッチは3つのパラメータで指定できます：

- :attr:`nworld <mujoco_warp.Data.nworld>`: シミュレートするワールドの数。
- _`nconmax`: ワールドごとの期待される接触数。すべてのワールドの最大接触数は ``nconmax * nworld`` です。
- _`naconmax`: `nconmax`_ の代替、すべてのワールドにわたる最大接触数。 `nconmax`_ と `naconmax`_ の両方が設定されている場合、 `nconmax`_ は無視されます。
- _`njmax`: ワールドごとの最大制約数。

.. admonition:: `nconmax`_ と `njmax`_ の意味の違い。
  :class: note

  ワールドごとの接触数が `nconmax`_ を超えることは可能ですが、すべてのワールドの総接触数が ``nworld x nconmax`` を超えない場合に限ります。
  ただし、ワールドごとの制約数は `njmax`_ によって厳密に制限されます。

.. admonition:: XMLパース
  :class: note

  `nconmax`_ と `njmax`_ の値は :ref:`size/nconmax <size-nconmax>` と
  :ref:`size/njmax <size-njmax>` からパースされません（これらのパラメータは非推奨です）。これらのパラメータの値は
  :func:`mjw.make_data <mujoco_warp.make_data>` または :func:`mjw.put_data <mujoco_warp.put_data>` に提供する必要があります。

関数
----

MuJoCo関数は、 `PEP 8 <https://peps.python.org/pep-0008/>`__ 準拠の名前に従ったMJWarp関数として公開されています。
ほとんどの :ref:`メインシミュレーション <Mainsimulation>` と
一部の :ref:`サブコンポーネント <Subcomponents>` の順シミュレーションがトップレベルの
:mod:`mjw <mujoco_warp>` モジュールから利用できます。

最小限の例
----------

.. code-block:: python

   # 100種類の速度でボールを投げる。

   import mujoco
   import mujoco_warp as mjw
   import warp as wp

   _MJCF=r"""
   <mujoco>
     <worldbody>
       <body>
         <freejoint/>
         <geom size=".15" mass="1" type="sphere"/>
       </body>
     </worldbody>
   </mujoco>
   """

   mjm = mujoco.MjModel.from_xml_string(_MJCF)
   m = mjw.put_model(mjm)
   d = mjw.make_data(mjm, nworld=100)

   # initialize velocities
   wp.copy(d.qvel, wp.array([[float(i) / 100, 0, 0, 0, 0, 0] for i in range(100)], dtype=float))

   # simulate physics
   mjw.step(m, d)

   print(f'qpos:\n{d.qpos.numpy()}')

.. _mjwCLI:

コマンドラインスクリプト
------------------------

_`testspeed` で環境をベンチマーク

.. code-block:: shell

    mjwarp-testspeed benchmark/humanoid/humanoid.xml

MJWarpでインタラクティブな環境シミュレーション

.. code-block:: shell

    mjwarp-viewer benchmark/humanoid/humanoid.xml

機能の互換性
============

MJWarpは、いくつかの例外を除き、MuJoCoのほとんどのメインシミュレーション機能をサポートしています。MJWarpは、サポートされていない機能を参照するフィールド値を持つ :ref:`mjModel` をデバイスにコピーするように求められた場合、例外を発生させます。

以下の機能はMJWarpで **サポートされていません** ：

.. list-table::
   :width: 90%
   :align: left
   :widths: 2 5
   :header-rows: 1

   * - カテゴリ
     - 機能
   * - :ref:`Integrator <mjtIntegrator>`
     - ``IMPLICIT``, ``IMPLICITFAST`` は流体抵抗と一緒にサポートされていません
   * - :ref:`Solver <mjtSolver>`
     - ``PGS``, ``noslip``, :ref:`islands <soIsland>`
   * - Fluid Model
     - :ref:`flEllipsoid`
   * - :ref:`Sensors <mjtSensor>`
     - ``GEOMDIST``, ``GEOMNORMAL``, ``GEOMFROMTO``
   * - Flex
     - ``VERTCOLLIDE=false``, ``INTERNAL=true``
   * - Jacobian format
     - ``SPARSE``
   * - Option
     - :ref:`contact override <COverride>`
   * - Plugins
     - ``All`` except ``SDF``
   * - :ref:`User parameters <CUser>`
     - ``All``

.. _mjwPerf:

パフォーマンスチューニング
==========================

以下はMJWarpのパフォーマンスを最適化するための考慮事項です。

.. _mjwGC:

グラフキャプチャ
----------------

MJWarp関数、例えば :func:`mjw.step <mujoco_warp.step>` は、しばしばカーネル起動のコレクションで構成されています。
Warpは、関数が直接呼び出された場合、これらのカーネルを個別に起動します。パフォーマンスを向上させるために、特に関数が複数回呼び出される場合、
関数を構成する操作をCUDAグラフとしてキャプチャすることをお勧めします。

.. code-block:: python

  with wp.ScopedCapture() as capture:
    mjw.step(m, d)

グラフは起動または再起動できます

.. code-block:: python

  wp.capture_launch(capture.graph)

そして通常、関数を直接呼び出すよりも大幅に高速になります。詳細については、
`Warp Graph APIリファレンス <https://nvidia.github.io/warp/modules/runtime.html#graph-api-reference>`__ を参照してください。

バッチサイズ
------------

接触と制約の最大数、 `nconmax`_ / `naconmax`_ と `njmax`_ は、
:func:`mjw.make_data <mujoco_warp.make_data>` または
:func:`mjw.put_data <mujoco_warp.put_data>` で :class:`mjw.Data <mujoco_warp.Data>` を作成するときに指定されます。メモリと計算はこれらのパラメータの値でスケールします。
最高のパフォーマンスのために、これらのパラメータの値は、シミュレーションがこれらの制限を超えないようにしながら、可能な限り小さく設定する必要があります。

これらの制限の適切な値は環境固有であることが予想されます。実際には、適切な値を選択することは通常、試行錯誤を伴います。
:func:`mjwarp-testspeed <mujoco_warp.testspeed>` を `--measure_alloc` フラグで使用して、各シミュレーションステップでの接触数と制約数を出力し、
:func:`mjwarp-viewer <mujoco_warp.viewer>` を介してシミュレーションと対話してオーバーフローエラーをチェックすることは、これらのパラメータの値を反復的にテストするための有用な技術です。

ソルバー反復
------------

MuJoCoのデフォルトのソルバー設定は、 :ref:`ソルバー反復 <option-iterations>` の最大数と
:ref:`直線探索反復 <option-ls_iterations>` の最大数について、妥当なパフォーマンスを提供することが期待されます。MJWarpの設定
:attr:`Option.iterations <mujoco_warp.Option.iterations>` および/または
:attr:`Option.ls_iterations <mujoco_warp.Option.ls_iterations>` 制限を減らすことでパフォーマンスが向上する可能性があり、
`nconmax`_ / `naconmax`_ と `njmax`_ のチューニング後の二次的な考慮事項となるべきです。

これらの制限を減らしすぎると、制約ソルバーが収束しなくなり、不正確または不安定なシミュレーションにつながる可能性があります。

.. admonition:: パフォーマンスへの影響：MJX (JAX) とMJWarp
  :class: note

  :ref:`MJX<mjx>` では、これらのソルバーパラメータはシミュレーションパフォーマンスを制御する鍵となります。対照的に、MJWarpでは、
  すべてのワールドが収束するとソルバーは早期終了し、不必要な計算を回避できます。その結果、これらの設定の値はパフォーマンスへの影響が比較的少なくなります。

接触センサーマッチング
----------------------

:ref:`接触センサー <sensor-contact>` を含むシーンには、センサーごとの最大マッチング接触数を指定するパラメータ
:attr:`Option.contact_sensor_max_match <mujoco_warp.Option.contact_sensor_max_match>` があります。
最高のパフォーマンスのために、このパラメータの値は、シミュレーションが制限を超えないようにしながら、可能な限り小さくする必要があります。
この制限を超えるマッチング接触は無視されます。

このパラメータの値は、例えば ``model.opt.contact_sensor_maxmatch = 16`` のように直接設定するか、XML
カスタム数値フィールド経由で設定できます。

.. code-block:: xml

   <custom>
     <numeric name="contact_sensor_maxmatch" data="16"/>
   </custom>

接触と制約の最大数と同様に、この設定の適切な値は環境固有であることが期待されます。
:func:`mjwarp-testspeed <mujoco_warp.testspeed>` と :func:`mjwarp-viewer <mujoco_warp.viewer>` は、このパラメータの値をチューニングするのに役立つ可能性があります。

並列直線探索
------------

制約ソルバーの反復的な直線探索に加えて、MJWarpはステップサイズのセットを並列に評価して最適なものを選択する並列直線探索ルーチンを提供します。
ステップサイズは :attr:`Model.opt.ls_parallel_min_step <mujoco_warp.Option.ls_parallel_min_step>` から1まで対数的に間隔が空けられ、
評価するステップサイズの数は :attr:`Model.opt.ls_iterations <mujoco_warp.Option.ls_iterations>` で設定されます。

場合によっては、並列ルーチンが制約ソルバーのデフォルトの反復的な直線探索と比較して、改善されたパフォーマンスを提供する可能性があります。

このルーチンを有効にするには ``Model.opt.ls_parallel=True`` を設定するか、XMLにカスタム数値フィールドを追加します。

.. code-block:: xml

   <custom>
     <numeric name="ls_parallel" data="1"/>
   </custom>

.. admonition:: 実験的機能
  :class: note

  並列直線探索は現在実験的な機能です。

.. _mjwBatch:


メモリ
------

シミュレーションスループットは、大量のワールドに対するメモリ要件によって制限されることがよくあります。メモリ使用量を最適化するための考慮事項は以下の通りです：

- CCDコライダーはプリミティブコライダーよりも多くのメモリを必要とします。コライダーに関する情報はMuJoCoの :ref:`ペアワイズコライダーテーブル <coPairwise>` を参照してください。
- :ref:`multiccd <option-flag-multiccd>` はCCDよりも多くのメモリを必要とします。
- CCDのメモリ要件は :ref:`Option.ccd_iterations <option-ccd_iterations>` に線形にスケールします。
- 少なくとも1つのメッシュジオムを持ち、 :ref:`multiccd <option-flag-multiccd>` を使用するシーンは、
  面ごとの最大頂点数と頂点ごとの最大エッジ数に線形にスケールするメモリ要件を持ち、すべてのメッシュにわたって計算されます。

`testspeed`_ は、シミュレーションの総メモリ使用量と、大量のメモリを必要とする
:class:`mjw.Model <mujoco_warp.Model>` および :class:`mjw.Data <mujoco_warp.Data>` フィールドに関する情報を報告するための ``--memory`` フラグを提供します。
CCDと制約ソルバーのためのインラインに割り当てられるメモリも重要であり、 ``Other memory`` として報告されます。

.. admonition:: コライダーごとの最大接触数
  :class: note

  一部のMJWarpコライダーは、MuJoCoと比較して異なる最大接触数を持っています：

  - ``PLANE<>MESH``: 4 対 3
  - ``HFieldCCD``: 4 対 ``mjMAXCONPAIR``

.. admonition:: スパース性
  :class: note

  スパースヤコビアンは大幅なメモリ削減を可能にします。この機能の更新はGitHub issue
  `#88 <https://github.com/google-deepmind/mujoco_warp/issues/88>`__ で追跡されています。

バッチ化された :class:`Model <mujoco_warp.Model>` フィールド
==================================================================

異なるモデルパラメータ値を使用したバッチシミュレーションを可能にするために、多くの :class:`mjw.Model <mujoco_warp.Model>` フィールドは
先頭にバッチ次元を持ちます。デフォルトでは、先頭次元は1（つまり、 ``field.shape[0] == 1`` ）であり、同じ値がすべてのワールドに適用されます。
これらのフィールドの1つを、先頭次元が1より大きい ``wp.array`` でオーバーライドすることが可能です。このフィールドは、ワールドIDとバッチ次元の
剰余演算でインデックス付けされます： ``field[worldid % field.shape[0]]`` 。

.. admonition:: グラフキャプチャ
  :class: warning

  フィールド配列は
  :ref:`グラフキャプチャ <mjwGC>` （つまり、 ``wp.ScopedCapture`` ）の前にオーバーライドする必要があります。
  更新は既存のグラフに適用されないためです。

.. code-block:: python

   # override shape and values
   m.dof_damping = wp.array([[0.1], [0.2]], dtype=float)

   with wp.ScopedCapture() as capture:
     mjw.step(m, d)

グラフキャプチャ後にフィールドの形状をオーバーライドし、フィールド値を設定することが可能です。

.. code-block:: python

   # override shape
   m.dof_damping = wp.empty((2, 1), dtype=float)

   with wp.ScopedCapture() as capture:
     mjw.step(m, d)

   # set batched values
   dof_damping = wp.array([[0.1], [0.2]], dtype=float)
   wp.copy(m.dof_damping, dof_damping)  # m.dof = dof_damping will not update the captured graph

フィールドの変更
----------------

:ref:`mjModel` フィールドを変更するための推奨ワークフローは、最初に対応する :ref:`mjSpec` を変更してから、
コンパイルして更新されたフィールドを持つ新しい :ref:`mjModel` を作成することです。ただし、コンパイルは現在ホスト呼び出しを必要とします：
新しいフィールドインスタンスごとに1回の呼び出し、つまり ``nworld`` インスタンスに対して ``nworld`` 回のホスト呼び出しです。

特定のフィールドはコンパイルなしで直接変更しても安全であり、デバイス上での更新が可能です。特定のフィールドの詳細については、
:ref:`mjModelの変更 <sichange>` を参照してください。さらに、
`GitHub issue 893 <https://github.com/google-deepmind/mujoco_warp/issues/893>`__ は、フィールドのサブセットに対するデバイス上での更新の追加を追跡しています。

.. admonition:: 不均質なワールド
   :class: note

   不均質なワールド、例えば：ワールドごとのメッシュや自由度の数は、現在利用できません。

.. _mjwFAQ:

よくある質問
============

学習フレームワーク
------------------

**MJWarpはJAXで動作しますか？**

はい。MJWarpは `JAX <https://jax.readthedocs.io/>`__ と相互運用可能です。詳細については、
`Warp Interoperability <https://nvidia.github.io/warp/modules/interoperability.html#jax>`__ ドキュメントを参照してください。

さらに、 :ref:`MJX <mjx>` はMJWarpの :doc:`API <api>` のサブセットに対してJAX APIを提供します。実装は
``impl='warp'`` で指定されます。

**MJWarpはPyTorchで動作しますか？**

はい。MJWarpは `PyTorch <https://pytorch.org>`__ と相互運用可能です。詳細については、
`Warp Interoperability <https://nvidia.github.io/warp/modules/interoperability.html#pytorch>`__ ドキュメントを参照してください。

**MJWarp物理で方策を訓練するには？**

MJWarp物理で方策を訓練する例については、以下を参照してください：

- `Isaac Lab <https://github.com/isaac-sim/IsaacLab/tree/feature/newton>`__: `Newton API <https://github.com/newton-physics/newton>`__ 経由で訓練します。
- `mjlab <https://github.com/mujocolab/mjlab>`__: PyTorchを使用してMJWarpで直接訓練します。
- `MuJoCo Playground <https://github.com/google-deepmind/mujoco_playground>`__: :ref:`MJX API <mjx>` 経由で訓練します。

機能
----

.. _mjwDiff:

**MJWarpは微分可能ですか？**

いいえ。MJWarpは現在、
Warpの `自動微分 <https://nvidia.github.io/warp/modules/differentiability.html#differentiability>`__ 機能を介して微分可能ではありません。
MJWarpの自動微分を有効にすることに関するチームからの更新はこちらの
`GitHub issue <https://github.com/google-deepmind/mujoco_warp/issues/500>`__ で追跡されています。

**MJWarpは複数のGPUで動作しますか？**

はい。Warpの ``wp.ScopedDevice`` はマルチGPU計算を可能にします。

.. code-block:: python

   # create a graph for each device
   graph = {}
   for device in wp.get_cuda_devices():
     with wp.ScopedDevice(device):
       m = mjw.put_model(mjm)
       d = mjw.make_data(mjm)
       with wp.ScopedCapture(device) as capture:
         mjw.step(m, d)
       graph[device] = capture.graph

   # launch a graph on each device
   for device in wp.get_cuda_devices():
     wp.capture_launch(graph[device])

詳細については、
`Warp ドキュメント <https://nvidia.github.io/modules/devices.html#example-using-wp-scopeddevice-with-multiple-gpus>`__ および
`mjlab 分散訓練 <https://github.com/mujocolab/mjlab/tree/main/docs/api/distributed_training.md>`__ の強化学習の例を参照してください。

**GPU上のMJWarpは決定論的ですか？**

いいえ。同じコードの異なる実行によって計算される結果の間に、順序付けまたは *わずかな* 数値の違いがある可能性があります。
これはGPU上の非決定論的アトミック操作の特性です。決定論的な結果を得るには、 ``wp.set_device("cpu")`` でデバイスをCPUに設定してください。

GPU上での決定論的な結果の開発はこちらの
`GitHub issue <https://github.com/google-deepmind/mujoco_warp/issues/562>`__ で追跡されています。

**方向はどのように表現されますか？**

方向は単位クォータニオンとして表現され、 :ref:`MuJoCoの慣例 <siLayout>` に従います：
``w, x, y, z`` または ``スカラー、ベクトル`` 。

.. admonition:: ``wp.quaternion``
  :class: note

  MJWarpはWarpの `組み込み型 <https://nvidia.github.io/warp/modules/functions.html#warp.quaternion>`__
  ``wp.quaternion`` を使用します。ただし重要なことに、MJWarpはWarpの ``x, y, z, w`` クォータニオン慣例や
  操作を使用せず、代わりにMuJoCoの慣例に従うクォータニオンルーチンを実装しています。実装については、
  `math.py <https://github.com/google-deepmind/mujoco_warp/blob/main/mujoco_warp/_src/math.py>`__ を参照してください。

**MJWarpには名前付きアクセスAPI / bindがありますか？**

いいえ。この機能の更新はこちらの
`GitHub issue <https://github.com/google-deepmind/mujoco_warp/issues/884>`__ で追跡されています。

**衝突がないのに接触が報告されるのはなぜですか？**

衝突センサーに寄与する各ユニークなジオムペアに対して、そのジオムペアが衝突していない場合でも、1つの接触が報告されます。
:ref:`衝突センサー <collision-sensors>` がセンサーデータを計算する際に衝突ルーチンへの別個の呼び出しを行うMuJoCoまたはMJXとは異なり、
MJWarpはメイン衝突パイプラインの実行中にこれらのセンサーのデータを接触で計算して保存します。

:ref:`接触センサー <sensor-contact>` は、物理に影響を与える接触に関する正しい情報を報告します。

**ヤコビアンが常に密なのはなぜですか？**

スパースヤコビアンは現在実装されておらず、 ``Data`` フィールド： ``ten_J`` 、 ``actuator_moment`` 、 ``flexedge_J`` 、
``efc.J`` は常に密行列として表現されます。スパースヤコビアンのサポートはGitHub issue
`#88 <https://github.com/google-deepmind/mujoco_warp/issues/88>`__ で追跡されています。

**一部の配列がmjModelまたはmjDataと比較して異なる形状を持つのはなぜですか？**

バッチシミュレーションのデフォルトでは、多くの :class:`mjw.Data <mujoco_warp.Data>` フィールドは ``Data.nworld`` サイズの
先頭バッチ次元を持ちます。一部の :class:`mjw.Model <mujoco_warp.Model>` フィールドは、サイズ ``1`` の先頭バッチ次元を持ち、
これは :ref:`ドメインランダム化のためのバッチパラメータの配列でフィールドをオーバーライドできる <mjwBatch>` ことを示しています。

さらに、 ``Model.qM`` 、 ``Data.efc.J`` 、 ``Data.efc.D`` を含む特定のフィールドは、GPU上での高速読み込みを可能にするためにパディングされています。

**MJWarpとMuJoCoの数値結果が異なるのはなぜですか？**

MJWarpは、 :ref:`mjtNum` のデフォルトのdouble表現を使用するMuJoCoとは対照的に、
`float <https://nvidia.github.io/warp/modules/functions.html#warp.float32>`__ を使用します。反復、衝突検出、小さな摩擦値を含むソルバー設定は、
浮動小数点表現の違いに敏感である可能性があります。

NaNを含む予期しない結果に遭遇した場合は、GitHub issueを開いてください。

**慣性行列qMのスパース性がMuJoCo / MJXと一致しないのはなぜですか？**

.. admonition:: ``mjtJacobian`` の意味
   :class: note

   - MuJoCoの慣性行列は常にスパースであり、 :ref:`mjtJacobian` は制約ヤコビアンと関連する量に影響します
   - MJWarp（およびMJX）の制約ヤコビアンは常に密であり、 :ref:`mjtJacobian` は密またはスパースとして表現できる慣性行列に影響するように再利用されます

MJWarpが ``AUTO`` に使用する自動スパース性しきい値はGPU用に最適化されており、 ``nv > 32`` に設定されています。
これは ``nv >= 60`` を使用するMuJoCoおよびMJXとは異なります。密 ``DENSE`` およびスパース ``SPARSE`` 設定は、MuJoCoおよびMJXと一致しています。

この機能は将来変更される可能性があります。

**シミュレーション実行時警告を修正するには？**

シミュレーション中にメモリ要件が既存の割り当てを超えた場合に警告が提供されます：

- `nconmax`_ / `njmax`_: 接触/制約の最大数が超過しました。 :func:`mjw.make_data <mujoco_warp.make_data>` または
  :func:`mjw.put_data <mujoco_warp.put_data>` への関連引数を更新して設定値を増やしてください。
- ``mjw.Option.ccd_iterations``: 凸衝突検出アルゴリズムが最大反復数を超えました。
  XML / :ref:`mjSpec` / :ref:`mjModel` でこの設定の値を増やしてください。重要なことに、この変更は
  :func:`mjw.put_model <mujoco_warp.put_model>` および
  :func:`mjw.make_data <mujoco_warp.make_data>` / :func:`mjw.put_data <mujoco_warp.put_data>` に提供される :ref:`mjModel` インスタンスに対して行う必要があります。
- ``mjw.Option.contact_sensor_maxmatch``: :ref:`接触センサー <sensor-contact>` のマッチング基準に対する最大接触マッチ数が超過しました。
  このMJWarp専用設定 `m.opt.contact_sensor_maxmatch` の値を増やしてください。または、接触センサーのマッチング基準をリファクタリングしてください。
  例えば、対象の2つのジオムがわかっている場合は、 ``geom1`` と ``geom2`` を指定してください。
- ``height field collision overflow``: ハイトフィールドによって生成される潜在的な接触数が
  :ref:`mjMAXCONPAIR <glNumeric>` を超え、一部の接触が無視されます。この警告を解決するには、ハイトフィールドの解像度を下げるか、
  ハイトフィールドと相互作用するジオムのサイズを小さくしてください。

コンパイル
----------

**コンパイル時間を改善するには？**

一般的な凸衝突パイプラインを必要とするユニークなコライダーの数を制限してください。これらのコライダーは
`collision_convex.py <https://github.com/google-deepmind/mujoco_warp/blob/main/mujoco_warp/_src/collision_convex.py>`__ で ``_CONVEX_COLLISION_PAIRS`` としてリストされています。
パイプラインのコンパイル時間の改善はこちらの
`GitHub issue <https://github.com/google-deepmind/mujoco_warp/issues/813>`__ で追跡されています。

**MJWarpをアップグレードした後、物理が期待通りに動作しないのはなぜですか？**

Warpキャッシュが現在のコードと互換性がない可能性があり、デバッグプロセスの一部としてクリアする必要があります。これは、
ディレクトリ ``~/.cache/warp`` を削除するか、Python経由で実行できます。

.. code-block:: python

   import warp as wp
   wp.clear_kernel_cache()

**実行時ではなく、事前にMJWarpをコンパイルすることは可能ですか？**

はい。詳細については、Warpの
`Ahead-of-Time Compilation Workflows <https://nvidia.github.io/warp/codegen.html#ahead-of-time-compilation-workflows>`__
ドキュメントを参照してください。

MuJoCoとの違い
==============

このセクションでは、MJWarpとMuJoCoの違いを説明します。

ウォームスタート
----------------

ウォームスタートが :ref:`無効化されていない <option-flag-warmstart>` 場合、MJWarpソルバーのウォームスタートは常に
``qacc_warmstart`` で加速度を初期化します。対照的に、MuJoCoは ``qacc_smooth`` と
``qacc_warmstart`` の間の比較を実行して、どちらが初期化に使用されるかを決定します。

慣性行列の分解
--------------

密計算を使用する場合、MJWarpの慣性行列の分解 ``qLD`` はWarpの ``L'L``
コレスキー分解
`wp.tile_cholesky <https://nvidia.github.io/warp/language_reference/_generated/warp._src.lang.tile_cholesky.html>`__ で計算され、
結果は異なる逆モード ``L'DL`` ルーチン
:ref:`mj_factorM` が使用されるため、MuJoCoの対応するフィールドと一致することは期待されていません。

オプション
----------

:class:`mjw.Option <mujoco_warp.Option>` フィールドは :ref:`mjOption` の対応するフィールドと一致しますが、以下の例外があります：

- :ref:`impratio <option-impratio>` はその逆平方根 ``impratio_invsqrt`` として格納されます。
- 制約ソルバー設定の :ref:`tolerance <option-tolerance>` は最小値 ``1e-6`` にクランプされます。
- 接触の :ref:`オーバーライド <option-flag-override>` パラメータ :ref:`o_margin <option-o_margin>` 、 :ref:`o_solref <option-o_solref>` 、 :ref:`o_solimp <option-o_solimp>` 、および :ref:`o_friction <option-o_friction>` は利用できません。

:ref:`disableflags <option-flag>` には以下の違いがあります：

- :ref:`mjDSBL_MIDPHASE <mjtDisablebit>` は利用できません。
- :ref:`mjDSBL_AUTORESET <mjtDisablebit>` は利用できません。
- :ref:`mjDSBL_NATIVECCD <mjtDisablebit>` はデフォルトのボックス-ボックスコライダーをCCDからプリミティブコライダーに変更します。
- :ref:`mjDSBL_ISLAND <mjtDisablebit>` は現在利用できません。制約アイランドの発見はGitHubイシュー `#886 <https://github.com/google-deepmind/mujoco_warp/issues/886>`__ で追跡されています。

:ref:`enableflags <option-flag>` には以下の違いがあります：

- :ref:`mjENBL_OVERRIDE <mjtEnablebit>` は利用できません。
- :ref:`mjENBL_FWDINV <mjtEnablebit>` は利用できません。
- :ref:`mjENBL_ISLAND <mjtEnablebit>` による制約アイランドスリーピングは現在利用できません。この機能はGitHubイシュー `#886 <https://github.com/google-deepmind/mujoco_warp/issues/886>`__ と `#887 <https://github.com/google-deepmind/mujoco_warp/issues/887>`__ で追跡されています。

MJWarp固有の追加オプションが利用可能です：

- ``ls_parallel``: 制約ソルバーで並列直線探索を使用
- ``ls_parallel_min_step``: 並列直線探索の最小ステップサイズ
- ``broadphase``: ブロードフェーズアルゴリズムのタイプ（ :class:`mjw.BroadphaseType <mujoco_warp.BroadphaseType>` ）
- ``broadphase_filter``: ブロードフェーズで使用されるフィルタリングのタイプ（ :class:`mjw.BroadphaseFilter <mujoco_warp.BroadphaseFilter>` ）
- ``graph_conditional``: CUDAグラフ条件を使用
- ``run_collision_detection``: 衝突検出ルーチンを使用
- ``contact_sensor_maxmatch``: 接触センサーのマッチング基準の最大接触数

.. admonition:: 流体モデル
  :class: note

  流体モデルパラメータ ``density`` 、 ``viscosity`` 、または ``wind`` を変更する場合、 ``Model.has_fluid`` の更新が必要な場合があります。

.. admonition:: グラフキャプチャ
  :class: note

  更新された設定を有効にするために、 :class:`mjw.Option <mujoco_warp.Option>` フィールドの変更後に新しい :ref:`グラフキャプチャ <mjwGC>` が必要な場合があります。
