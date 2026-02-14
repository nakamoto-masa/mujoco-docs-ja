=========
変更履歴
=========

Version 3.5.0 (February 12, 2026)
---------------------------------

主要な新機能
^^^^^^^^^^^^^^^^^^^^^^^^

1. :doc:`MuJoCo Warp <mjwarp/index>` が正式にリリースされました。
2. 新しい **システム同定** ツールボックス（Python）が追加されました。詳細は `README <https://github.com/google-deepmind/mujoco/blob/main/python/mujoco/sysid/README.md>`__ を参照してください。
   |br| ツールボックスのデモColabノートブックはこちらから利用できます: |sysid_colab|
   |br| :github:user:`kevinzakka` 、 :github:user:`aftersomemath` 、 :github:user:`jonathanembleyriches` 、 :github:user:`qiayuanl` 、 :github:user:`spjardim` および :github:user:`gizemozd` による貢献。

.. |sysid_colab| image:: https://colab.research.google.com/assets/colab-badge.png
                 :target: https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/python/mujoco/sysid/sysid.ipynb


3. アクチュエータとセンサーが履歴バッファを介した任意のディレイをサポートするようになり、センサー値をシミュレーションのタイムステップより大きなインターバルで計算できるようになりました。ディレイまたはインターバルを使用すると、新しい ``mjData.history`` 変数が :ref:`物理状態<siPhysicsState>` に追加されます。詳細は :ref:`ディレイ<CDelay>` を参照してください。

.. image:: images/changelog/poncho.png
   :width: 45%
   :align: right
   :target: https://github.com/google-deepmind/mujoco/blob/main/model/flex/poncho.xml

4. よりコースなメッシュでの布シミュレーションを可能にする新しい :ref:`flexvert<equality-flexvert>` 等式制約が追加されました。これにより、flexcomp エッジの :ref:`equality<flexcomp-edge-equality>` に新しい属性値 ``vert`` と、新しい等式制約タイプ :ref:`flexvert<equality-flexvert>` が追加されます。 `Chen, Kry and Vouga, 2019 <https://arxiv.org/abs/1911.05204>`__ で説明されている手法を使用しています。
5. ``implicit`` および ``implicitfast`` :ref:`積分器<geIntegration>` における変形可能オブジェクト（flex）の陰的積分サポートが追加されました。この手法はflexの自由度を抽出し、密なブロックとして解くことで、タイムステップを減少させることなく剛性の高いflexオブジェクトの安定性を向上させます。 ``trilinear`` および ``quadratic`` :ref:`dof<body-flexcomp-dof>` タイプと互換性があります。

.. image:: images/XMLreference/rfcamera.png
   :width: 45%
   :align: right
   :target: https://github.com/google-deepmind/mujoco/blob/main/test/engine/testdata/sensor/rfcamera.xml

6. 測距計センサーを :ref:`rangefinder/camera<sensor-rangefinder-camera>` 属性を使用してカメラにアタッチできるようになりました。この場合、センサーは :ref:`camera/resolution<body-camera-resolution>` 属性を尊重し、各ピクセルに対して1本ずつ、複数のレイをキャストします。
7. :ref:`測距計<sensor-rangefinder>` が、レイ距離以外にも表面法線や交差点を含むさまざまな種類の情報を報告できるようになりました。

.. container:: custom-clear

   .. raw:: html

      <div style="clear: both;"></div>

全般
^^^^^^^

.. admonition:: 破壊的API変更
   :class: attention

   8. レイキャスト関数がオプションでレイ交差点における表面法線を計算するようになりました。これは ``mjtNum normal[3]`` 引数の追加による破壊的変更です。変更された関数は :ref:`mj_ray` 、 :ref:`mj_multiRay` 、 :ref:`mju_rayGeom` 、 :ref:`mj_rayFlex` 、 :ref:`mj_rayHfield` および :ref:`mj_rayMesh` です。

      **移行方法:** C/C++では、 ``normal`` 引数に ``NULL`` を渡してください。Pythonでは、 :ref:`mj_multiRay` を除くすべての関数でデフォルト値が ``None`` であるため、対応は不要です。

   9. ``mju_rayFlex`` は、 ``mjModel*`` および ``mjData*`` 引数を取る他の関数との一貫性のために :ref:`mj_rayFlex` に名前が変更されました。

   10. ``mjModel.cam_orthographic`` フィールドは ``cam_projection`` に名前が変更され、新しい列挙型 :ref:`mjtProjection` のセマンティクスを持つようになりました。これにより、将来的に魚眼カメラなどのより多くの投影タイプが可能になります。関連して、カメラの ``camera/orthographic`` MJCF属性は :ref:`camera/projection<body-camera-projection>` に名前が変更され、 ``orthographic`` と ``perspective`` の値を受け付けるようになりました。

       **移行方法:** ``orthographic = "false/true"`` をそれぞれ ``projection="perspective/orthographic"`` に置き換えてください。

   11. ``mjpResourceProvider`` 構造体から ``getdir`` が削除されました。すべてのリソースプロバイダは同じ共有実装を使用するようになりました。
   12. 2つのジオムの ``margin`` または ``gap`` :ref:`パラメータ<CContact>` を組み合わせて接触のパラメータを取得する際、それぞれの値が最大値を取る代わりに **合計** されるようになりました。これにより、ジオムのマージンがジオムの適切な「膨張」となります。

13. カメラの錐台の可視化は、 :ref:`resolution<body-camera-resolution>` を1より大きい値に設定することでトリガーされるようになりました。関連して、錐台の可視化は :ref:`正射影<body-camera-projection>` カメラでも機能します。詳細は :ref:`rangefinder<sensor-rangefinder>` を参照してください。
14. カメラに :ref:`output<body-camera-output>` 属性が追加され、 ``mjModel.cam_output`` ビットフィールドに解析されます。レンダラーでは使用されませんが、カメラがサポートする出力タイプを格納する便利な場所として機能します。
15. カスタムVFSプロバイダをマウントするための :ref:`mj_mountVFS` および :ref:`mj_unmountVFS` 関数が追加されました。マウントにより、プロバイダを使用して任意のパスでリソースを動的にオープン/読み取り/クローズできます。
16. 同一の属性を持つ連続した :ref:`衝突センサー<collision-sensors>` が計算を共有する最適化が削除されました。これにより、この最適化を利用していたモデルでは（おそらく軽微な）パフォーマンスの低下が生じます。パフォーマンスを回復するには、 :ref:`fromto<sensor-fromto>` を使用して他の値を手動で計算してください。 ``from = fromto[0:3]`` および ``to = fromto[3:6]`` とすると、 ``distance = norm(to-from)`` および ``normal = normalize(to-from)`` となります。
17. :doc:`OpenUSD <OpenUSD/index>`:

    - パーシングが実験的な段階から mjpDecoder プラグインに移行されました。（ドキュメントは準備中）
    - OpenUSDを `third_party_deps/openusd <https://github.com/google-deepmind/mujoco/tree/main/cmake/third_party_deps/openusd>`__ CMakeユーティリティプロジェクトでビルドできるようになりました。
    - ``USD_DIR`` はMuJoCo CMakeプロジェクトで使用されなくなりました。ビルド済みのUSDライブラリがある場合は代わりに ``pxr_DIR`` を使用してください。
    - ユーザーは ``PXR_PLUGINPATH_NAME`` 環境変数を設定する必要がなくなりました。MuJoCoはUSDプラグインを自動的に読み込みます。
18. 非破壊的なABI変更:

    - :ref:`mj_stateSize` および関連関数の ``sig`` （シグネチャ）引数の型が ``unsigned int`` から ``int`` に変更されました。この変更以前は、この関数に渡される無効な負の引数は暗黙のキャストにより無視されていましたが、現在は負の値でエラーがトリガーされます。
    - :ref:`depth<mjtRndFlag>` レンダリングフラグが追加されました。
    - :ref:`mjModel` の割り当てサイズが、より大きなシーンに対応するために32ビット整数から64ビット整数を使用するようになりました。


MJX
^^^
19. ``mjx.Data`` に ``actuator_length`` 、 ``cdof`` および ``cdof_dof`` フィールドが追加されました。
20. 複数のWarpグラフキャプチャモードをサポートする ``graph_mode`` 引数が ``put_model`` に追加されました。

ドキュメント
^^^^^^^^^^^^^
21. :ref:`プログラミング/シミュレーション<Simulation>` の章が全般的に改善されました。特に、 :ref:`状態<siStateControl>` に関する主要な議論がこの章に移動され、 :ref:`mjModelの変更<siChange>` に関するセクションが拡充されました。
22. :ref:`MJCFスキーマ<CSchema>` の使いやすさが、要素と属性へのリンクを含む折りたたみ可能なドロップダウンメニューにより改善されました。
23. MuJoCoのバージョン番号付けがセマンティックバージョニングに基づくようになりました。 `VERSIONING.md <https://github.com/google-deepmind/mujoco/blob/main/VERSIONING.md>`__ を参照してください。


バグ修正
^^^^^^^^^
24. :ref:`陰的積分器<geIntegrators>` の導関数で、力が :ref:`forcerange<actuator-general-forcerange>` によってクランプされた場合にアクチュエータ速度導関数が誤って計算されるバグを修正しました。
25. :ref:`陰的積分器<geIntegrators>` の導関数で、アクチュエータ速度導関数が :ref:`actearly<actuator-general-actearly>` フラグを考慮していなかったバグを修正しました。
26. :ref:`usethread<compiler-usethread>` コンパイラフラグ（デフォルトでオン）によって有効化されるマルチスレッドメッシュ処理が、実際にはこのフラグによって無効化されていました。このバグの修正により、メッシュを多用するモデルのコンパイルが利用可能なコア数に応じて（最大で）高速化されます。
27. :ref:`mj_rayFlex` および :ref:`mju_raySkin` の ``vertid`` 引数はnull許容としてマークされていましたが、実際にはそうではありませんでした。現在はnull許容になりました。
28. ジョイントを持たないボディがジョイントを持つ親ボディ内にネストされている場合に :ref:`gravcomp<body-gravcomp>` が無視されるバグを修正しました（ :issue:`3066` 、 :github:user:`Alex108306` による報告）。

Version 3.4.0 (December 5, 2025)
--------------------------------

全般
^^^^^^^

.. youtube:: vct493lGQ8Q
   :align: right
   :width: 35%

1. 主要な新機能として :ref:`スリーピングアイランド<Sleeping>` を導入しました。早期テストのための予備リリースです。詳細はドキュメントを参照してください。
2. :ref:`flexcomp/dof<body-flexcomp-dof>` に「quadratic」オプションを追加しました。この高速な :ref:`変形可能<CDeformable>` フレックスオブジェクトのタイプは「trilinear」オプションに似ていますが、曲面変形を含みます。
3. 解析中に名前の衝突がある場合にもエラーを発生させるようになりました。
4. 深くネストされたボディ階層を持つモデルを有効にするため、Windowsのスタックサイズを16MBに増加しました。
5. すべての運動学的なサブコンポーネントを統合する新しいパイプラインコンポーネント関数 :ref:`mj_fwdKinematics` を追加しました。関連して、 :ref:`シミュレーションパイプライン<Pipeline>` の章の冒頭に明確化のための表を追加しました。
6. :ref:`mj_getState` によって以前に返された状態のサブセットを、まず ``mjData`` に書き戻すことなく抽出できる新しい :ref:`mj_extractState` 関数を追加しました。
7. 状態コンポーネントをある ``mjData`` から別の ``mjData`` にコピーする新しい :ref:`mj_copyState` 関数を追加しました。
8. テンドンパスがPythonから ``MjsTendon.path`` を介してクエリできるようになりました。返されるオブジェクトはイテラブルで、インデックスを指定するとパス内の指定されたインデックスの ``MjsWrap`` が取得できます。
9. ``MjsWrap`` が以下を公開するようになりました:

   - ``type -> mujoco.mjtWrap``
   - ``target -> MjsSite|MjsJoint|MjsGeom|None``
   - ``sidesite -> MjsSite|None``
   - ``coef -> real``
   - ``divisor -> real``

10. 非破壊的なABI変更:

    - :ref:`mjtSize` は、将来の型昇格バグを回避するために ``uint64_t`` ではなく ``int64_t`` として定義されるようになりました。
    - :ref:`mj_sizeModel` は ``int`` ではなく :ref:`mjtSize` を返すようになりました。

MJX
^^^

11. ``warp-lang`` のオプション依存関係が1.10.0に更新されました。 ``pmap`` がMJXからMuJoCo Warpで動作するようになりました。

.. admonition:: Breaking ABI changes
   :class: attention

   - ``mjx.Model.tex_data`` は、この潜在的に大きな配列に対するvmappingを避けるため、jax.Arrayではなくnumpy ndarrayになりました。これにより、Madrona MJXの特定のユースケースが壊れる可能性がありますが、このコードパスのサポートは終了しています。ユーザーはWarpベースのバッチレンダラーに移行される予定です。


バグ修正
^^^^^^^^^

12. ボックス同士の距離計算のバグを修正しました。 :github:user:`nvtw` による報告。

Version 3.3.7 (October 13, 2025)
--------------------------------

全般
^^^^^^^

.. admonition:: Breaking API changes
   :class: attention

   1. mjSpec C APIフィールドの :ref:`meshdir<compiler-meshdir>` と :ref:`texturedir<compiler-texturedir>` が、それぞれ `compiler.meshdir <https://github.com/google-deepmind/mujoco/blob/0baac589993220095cf09e153f194f35ca0f0738/include/mujoco/mjspec.h#L154>`__
      と `compiler.texturedir <https://github.com/google-deepmind/mujoco/blob/0baac589993220095cf09e153f194f35ca0f0738/include/mujoco/mjspec.h#L155>`__
      に移動されました。後方互換性のため、旧フィールドはPython APIでは引き続き利用可能ですが、将来のリリースで削除される予定です。

      **移行方法:** ``meshdir`` と ``texturedir`` を ``compiler.meshdir`` と ``compiler.texturedir`` に置き換えてください。
   2. ``mjx.put_data`` および ``mjx.put_model`` から ``_full_compat`` を削除しました。
   3. ``mjx.make_data`` の ``nconmax`` および ``njmax`` フィールドのデフォルト値が-1から ``None`` に変更されました。 ``nconmax`` は将来のリリースで ``naconmax`` に置き換えられる予定です。


3. リミットが定義されているジョイントデコレータおよび空間テンドンで、現在の値（角度または長さ）がリミットを超えている場合、 :ref:`制約インピーダンス<soParameters>` :math:`d` を使用して既存の色と :ref:`visual/rgba/constraint<visual-rgba-constraint>` を混合することで再着色されるようになりました。空間テンドンについては、この可視化支援は :ref:`material<tendon-spatial-material>` が設定されておらず :ref:`rgba<tendon-spatial-rgba>` がデフォルトの場合にのみ有効です。
4. MJCFファイルから一意なアセット依存関係のリストを計算するための :ref:`mju_getXMLDependencies` を追加しました。
5. :ref:`mju_getXMLDependencies` の結果を表示するコマンドラインユーティリティを提供するコードサンプル ``dependencies`` を追加しました。
6. MuJoCoのコンパイルに必要な最低限のC++標準がC++20になりました。これは2023年からGoogle内部ではそうでしたが、CMakeの更新が忘れられていました。

.. admonition:: Breaking ABI changes
   :class: attention

   7. 属性 ``mjOption.apirate`` は未使用であったため削除されました。
   8. MJXの ``mjx.make_data`` の ``nconmax`` および ``njmax`` フィールドのデフォルト値が-1から ``None`` に変更されました。

MJX
^^^
9. :issue:`2508` を修正しました。 ``get_data_into`` 中に ``qLD`` の形状がmjModelと一致しませんでした。
10. MuJoCo Warpの ``io.py`` への更新を取り込み、すべての環境にわたる最大接触数の設定に ``nconmax`` の代わりに ``naconmax`` を使用するようになりました。

バグ修正
^^^^^^^^^
11. :issue:`2881` を修正しました。 :at:`fitaabb` がメッシュにオフセットを追加し、不正なフレーム変換を適用していました。また、ジオムをメッシュのAABBにフィットさせる意味を統一しました: AABBがメッシュのAABBを含む最小のジオムを見つけることを意味するようになりました。

Version 3.3.6 (September 15, 2025)
----------------------------------

全般
^^^^^^^
1. 制約アイランドの探索と構築は、以前は実験的機能でしたが、現在は :ref:`ドキュメント化<soIsland>` されデフォルトとして昇格しました。 :ref:`option/flag/island <option-flag-island>` で無効化できます。アイランド処理はモノリシックな制約ソルバーに対して厳密な改善であると想定しています。問題が発生した場合はお知らせください。
2. :ref:`接触センサー<sensor-contact>` の :at-val:`subtree1/subtree2` 指定が、ワールドの直接の子ボディだけでなく、任意のボディで利用可能になりました。

.. admonition:: Breaking API changes
   :class: attention

   3. ``mjData.qacc_warmstart`` の更新が、ソルバー呼び出し（ :ref:`mj_fwdConstraint` ）の最後から :ref:`mj_step` の最後に移動され、他のすべての状態変数と一緒に更新されるようになりました。この変更により :ref:`mj_forward` が完全に冪等になります。

      この変更以前は、 :ref:`mj_forward` を繰り返し呼び出すと制約ソルバーが収束していました。これは、後続の各呼び出しが以前に更新された ``qacc_warmstart`` の値から開始するためでした。実際、これはまさにビューワで発生していた動作で、ビューワはPAUSEモードで :ref:`mj_forward` を繰り返し呼び出します。

      **移行方法:** コードがこの動作に依存していた場合は、各 :ref:`mj_forward` の後に手動で更新することで回復できます: ``qacc_warmstart ← qacc`` 。この動作は :ref:`simulate<saSimulate>` で「Pause update」トグル（デフォルトではオフ）をクリックすることで利用可能です。

      さらに、この変更は :ref:`RK4 <geIntegrators>` 積分器の出力に数値的な影響があります。この変更以前は、4つのルンゲ・クッタのサブステップそれぞれの後に ``qacc_warmstart`` の更新が行われるため、RK4のソルバー収束が速くなっていましたが、原理に基づかない積分という代償がありました。この変更により、RK4の積分が原理に基づいた明確に定義されたものになります。このRK4への変更は実質的にバグ修正であるため、以前の動作への移行方法は提供されません。

   4. パッシブ力を無効化する ``mjDSBL_PASSIVE`` フラグが削除され、対応する :ref:`mjcf<option-flag-spring>` :ref:`属性<option-flag-damper>` を持つ :ref:`mjDSBL_SPRING<mjtDisableBit>` と :ref:`mjDSBL_DAMPER<mjtDisableBit>` に置き換えられました。各フラグは、ジョイントとテンドンのバネまたはダンパーのみをそれぞれ無効化します。両方のフラグが設定されている場合、重力補償、流体力、 :ref:`mjcb_passive` コールバックで計算される力、および :ref:`mjPLUGIN_PASSIVE<mjtPluginCapabilityBit>` ケイパビリティフラグが渡された :ref:`プラグイン<exPlugin>` で計算される力を含む、 **すべての** パッシブ力が無効化されます。

      **移行方法:** 両方のフラグを設定することで、以前のフラグの動作を回復できます。


.. admonition:: Breaking ABI changes
   :class: attention

   5. :ref:`mjtMouse` の ``mjMOUSE_SELECT`` フラグは使用されなくなったため削除されました。

   6. アイランド処理のデフォルトへの昇格に伴い、有効化フラグ ``mjENBL_ISLAND`` が削除され、無効化フラグ :ref:`mjDSBL_ISLAND <mjtDisableBit>` に変換されました。

7. 曲面参照構成を持つシェルのサポートが追加されました。この `サンプル <https://github.com/google-deepmind/mujoco/blob/main/model/flex/basket.xml>`__ を参照してください。
8. フレックスを含む :ref:`パッシブ<flex-contact-passive>` 接触の実験的オプションが追加されました。

9. :ref:`mesh/material <asset-mesh-material>` 属性を使用して、メッシュアセットにデフォルトのマテリアルを割り当てるサポートが追加されました。

MJX
^^^
10. ``ten_length`` をMJXのパブリックAPIに昇格しました。 ``mjx.tendon`` のWarpサポートを追加しました。

.. admonition:: Breaking API changes
   :class: attention

   11. ``ten_length`` が ``mjx.Data._impl.ten_length`` からパブリックフィールド ``mjx.Data.ten_length`` に移動されました。

バグ修正
^^^^^^^^^
12. アイランド処理が有効な場合に、PythonバインディングによってMjDataオブジェクトが正しくシリアライズされない潜在的なバグを修正しました。


Version 3.3.5 (August 8, 2025)
------------------------------

全般
^^^^^^^
1. オブジェクトがサイトのボリューム内にあるかどうかを確認するための :ref:`insidesite<sensor-insidesite>` センサーを追加しました。周囲の環境ロジックでイベントをトリガーするのに便利です。
2. ユーザー定義の基準に基づいて接触情報を報告する :ref:`contact<sensor-contact>` センサーを追加しました。 :el:`contact` センサーの目的は、接触関連の情報を固定サイズの配列で報告することです。これは学習ベースのエージェントへの入力や環境ロジックで役立ちます。
3. 2つのオブジェクト間の貫通深さを指定された点で測定し、接線フレームでの滑り速度を計測する :ref:`tactile<sensor-tactile>` センサーを追加しました。このセンサーはSDFとの衝突時にのみ触覚データを報告します。
4. SdfLibプラグインおよび `SdfLib <https://github.com/UPC-ViRVIG/SdfLib>`__ への依存関係を削除しました。SDFはmjModelでネイティブにサポートされるようになりました。
5. :ref:`mjvOption` から ``oct_depth`` を削除しました（未使用）。
6. ビルトインメッシュを作成する機能を追加しました。 :ref:`mesh/builtin<asset-mesh-builtin>` を参照してください。
7. MuJoCo Cにおける慣性計算が、新しい :ref:`パイプライン<piStages>` 関数 :ref:`mj_makeM` によって実行されるようになりました。この関数は :ref:`mj_crb` のComposite Rigid Bodyアルゴリズムと :ref:`テンドンアーマチュア<tendon-spatial-armature>` に関連する追加項を組み合わせたものです。慣性を計算するために :ref:`mj_crb` を使用しているコードは、代わりに :ref:`mj_makeM` を使用する必要があります。

.. admonition:: Breaking API changes
   :class: attention

   8. ``mjVIS_FLEXBVH`` 列挙値が削除されました。その機能は :ref:`mjVIS_MESHBVH<mjtVisFlag>` によって提供されるようになりました。

バグ修正
^^^^^^^^^
9. mjSpecをアタッチした後、子のオブジェクトリストに要素が欠落するバグを修正しました。これは、要求されたボディのツリーに属するオブジェクトのみをリストに追加していたことが原因でしたが、アタッチされたオブジェクトは親のツリーに属するためスキップされていました。
10. 衝突メッシュの凸包が :ref:`接触ペア<contact-pair>` 経由でのみ衝突可能な場合に計算されないバグを修正しました。

Python
^^^^^^
11. Linuxでは、ビルド済み配布パッケージ（wheel）が ``manylinux_2_28`` プラットフォームタグをターゲットにするようになりました。以前のMuJoCo wheelは、2024年6月にサポートが終了したCentOS 7ベースの ``manylinux2014`` をターゲットにしていました。

MJX
^^^
12. MJXのバックエンド実装としてWarpを追加しました。実装は ``mjx.put_model(m, impl='warp')`` および ``mjx.make_data(m, impl='warp')`` で指定できます。Warp実装にはCUDAデバイスと ``warp-lang`` のインストール（ ``pip install mujoco-mjx[warp]`` ）が必要です。この機能は「ベータ」版として提供されており、一部のバグが想定されています。

Version 3.3.4 (July 8, 2025)
----------------------------

.. admonition:: Breaking API changes
   :class: attention

   1. ``mjs_detachBody`` 関数と ``mjs_detachDefault`` 関数が :ref:`mjs_delete` に置き換えられました。
   2. Pythonの ``element.delete`` 関数が ``spec.delete(element)`` に置き換えられました。
   3. mjSpec C APIにおいて、 :ref:`mjs_setString` を使用した要素名の直接設定が、新しい関数 :ref:`mjs_setName` に置き換えられました。これにより、コンパイル時ではなく設定時に名前の衝突を確認でき、エラーをより早期に検出できます。関連して、 ``name`` 属性がすべてのmjs要素から削除されました。既知の問題: 解析中にはエラーが発生しません。
   4. MJXにおいて、 ``mjx.Option`` データクラスが ``mjx.Model`` および ``mjx.Data`` と同様にプライベートフィールドとパブリックフィールドを持つようになりました。このデータ構造の基盤実装の違いにより、一部のフィールドはパブリックに利用できなくなりました。

全般
^^^^^^^
4. :ref:`visual/global/cameraid<visual-global-cameraid>` を使用して、ビューワの初期カメラを設定するサポートが追加されました。
5. Pythonの :ref:`パッシブビューワ<PyViewerPassive>` の ``Sync`` メソッドで状態のみを同期するサポートが追加されました。これはパフォーマンスの向上に有用です。デフォルトの動作は変更されず、モデルとデータ全体をコピーします。

バグ修正
^^^^^^^^^
6. :ref:`テンドンアーマチュア<tendon-spatial-armature>` が存在する場合に逆動力学が正しく計算されていなかった問題を修正しました。
7. ``mjx.put_data`` において、C実装で ``actuator_moment`` が正しくコピーされていなかったバグを修正しました。

ドキュメント
^^^^^^^^^^^^^
8. 3.3.3の変更履歴に欠落していた項目のドキュメントを追加し、破壊的変更の性質を明確化しました。以下の項目3および4を参照してください。

Version 3.3.3 (June 10, 2025)
-----------------------------

全般
^^^^^^^
1. アイランドデータがメモリ連続になるよう、アイランド実装をリファクタリングしました。これにより、ソルバーでのアイランド処理が高速化され、NewtonソルバーとPGSソルバーの追加への道が開かれます（現在はCGのみサポート）。
2. :at:`shell` プラグインを削除しました。これは :ref:`flexcomp<body-flexcomp>` でサポートされるようになり、 :ref:`elastic2d<flexcomp-elasticity-elastic2d>` 属性に依存して有効化されます（デフォルトではオフ）。

.. admonition:: Breaking API changes
   :class: attention

   3. ライトの :ref:`directional<body-light-directional>` （ブーリアン）フィールドが、追加のライティングタイプに対応するための :ref:`type<body-light-type>` フィールド（ :ref:`mjtLightType<mjtLightType>` 型）に置き換えられました。

      **移行方法:** light/directional="false/true" をそれぞれ light/type="spot/directional" に置き換えてください。

   4. :ref:`mjtColorSpace` 列挙型と、テクスチャの色空間（線形または `sRGB <https://en.wikipedia.org/wiki/SRGB>`__ ）を指定できる関連する :ref:`colorspace<asset-texture-colorspace>` 属性を追加しました。このプロパティがPNGファイルから正しく読み取られるようになったため、sRGBを使用するテクスチャファイルは異なってレンダリングされます。

      **移行方法:** この変更前と同じ見た目にすべきテクスチャについては、 :ref:`colorspace<asset-texture-colorspace>` を「linear」に設定してください。

5. :ref:`mj_crb` の呼び出しと、3.3.1で導入された :ref:`テンドンアーマチュア<tendon-spatial-armature>` をサポートする追加ロジックを組み合わせた新しいサブコンポーネント :ref:`mj_makeM` を追加しました。従来の ``mjData.qM`` に加えて、 :ref:`mj_makeM` は同じ行列のCSR表現である ``mjData.M`` も計算します。
6. 互換性のあるmjSpecに、mjModel内の実数値配列をコピーバックする新しい関数 :ref:`mj_copyBack` を追加しました。
7. :ref:`fusestatic<compiler-fusestatic>` の、参照を含まないモデルに限定されていた制限を撤廃しました。fusestatic フラグは、参照されていないすべてのボディを融合し、参照されているボディは無視するようになりました。

Simulate
^^^^^^^^
8. ``mjv_sceneState`` 構造体が削除されました。この構造体は、Pythonビューワをパッシブモードで使用する際の ``mjModel`` と ``mjData`` の部分的な同期に使用されていました。この機能は、可視化に不要な配列をコピーしない :ref:`mjv_copyModel` と :ref:`mjv_copyData` によって提供されるようになりました。

.. image:: images/changelog/procedural_terrain_generation.png
   :width: 33%
   :align: right

Pythonバインディング
^^^^^^^^^^^^^^^^^^^^^^^

9. モデル編集チュートリアルにプロシージャル地形生成のサンプルを追加しました: |mjspec_colab|

MJX
^^^
10. テンドンアーマチュアを追加しました。

.. |mjspec_colab| image:: https://colab.research.google.com/assets/colab-badge.png
                  :target: https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/python/mjspec.ipynb

Version 3.3.2 (April 28, 2025)
------------------------------

MJX
^^^
1. 逆動力学を追加しました。
2. テンドンアクチュエータ力センサーを追加しました。
3. :issue:`2606` を修正し、 ``make_data`` が ``body_pos`` と ``body_quat`` から ``mocap_pos`` と ``mocap_quat`` をコピーするようにしました。

Version 3.3.1 (Apr 9, 2025)
----------------------------

.. admonition:: Breaking API changes
   :class: attention

   1. 内部フレックス接触 :ref:`internal flex contacts<flex-contact-internal>` を切り替えるフラグのデフォルト値が「true」から「false」に変更されました。この機能はユーザーにとって直感的でないことが判明しました。
   2. すべてのアタッチ関数（ ``mjs_attachBody`` 、 ``mjs_attachFrame`` 、 ``mjs_attachToSite`` 、 ``mjs_attachFrameToSite`` ）が削除され、単一の関数 :ref:`mjs_attach` に置き換えられました。

全般
^^^^^^^
3. テンドンアーマチュア :ref:`tendon armature<tendon-spatial-armature>` を追加しました：テンドン長さの変化に関連する慣性です。
4. :ref:`compiler/saveinertial<compiler-saveinertial>` フラグを追加しました。XMLに保存する際、すべてのボディに対して明示的な慣性節を書き込みます。
5. :ref:`composite<body-composite>` に :ref:`orientation<body-composite-quat>` 属性を追加しました。さらに、コンポジットがフレームの直接の子要素になれるようにしました。
6. テンドンアクチュエータ力リミット :ref:`tendon actuator force limits<tendon-spatial-actuatorfrclimited>` と テンドンアクチュエータ力センサー :ref:`tendon actuator force sensor<sensor-tendonactuatorfrc>` を追加しました。

MJX
^^^
7. テンドンアクチュエータ力リミットを追加しました。

バグ修正
^^^^^^^^^
8. :ref:`mj_jacDot` で、ヤコビアンが計算される点の運動を考慮する項が欠落していた問題を修正しました。
9. mjSpec をフレームまたはサイトにアタッチする際に、子ワールドボディ内の要素の親フレームが誤って設定されるバグを修正しました。
10. ARB_clip_controlをサポートしないプラットフォーム（例：MacOS）で影のレンダリングがちらつくバグを修正しました。 :github:user:`aftersomemath` との共同修正です。

Pythonバインディング
^^^^^^^^^^^^^^^^^^^^^^^

.. youtube:: LbANnKMDOHg
   :aspect: 16:7
   :align: right
   :width: 240px

11. モデル編集チュートリアルにプロシージャルモデル作成のサンプルを追加しました: |mjspec_colab_331|
12. ``bind`` メソッドで名前なしの :ref:`mjSpec` オブジェクトのサポートを追加しました。ドキュメントの対応する :ref:`セクション<PyMJCF>` を参照してください。

.. |mjspec_colab_331| image:: https://colab.research.google.com/assets/colab-badge.png
                      :target: https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/python/mjspec.ipynb

Version 3.3.0 (Feb 26, 2025)
----------------------------

機能の昇格
^^^^^^^^^^^^^^^^^
.. youtube:: qJFbx-FR7Bc
   :aspect: 16:7
   :align: right
   :width: 240px

1. :ref:`flexcomp/dof<body-flexcomp-dof>` を「trilinear」に設定することで有効化される、新しい種類の **高速変形可能ボディ** を導入しました。このタイプの :ref:`変形可能<CDeformable>` フレックスオブジェクトは、通常のフレックスと同じ衝突ジオメトリを持ちますが、自由度が大幅に少なくなっています。頂点ごとに3自由度を持つ代わりに、バウンディングボックスの角のみが自由に移動でき、内部頂点の位置は8つの角のトリリニア補間で計算されます。これにより、フレックスオブジェクト全体で合計24自由度（一部の角が固定されている場合はそれ以下）となります。これはフレックスで実現可能な変形の種類を制限しますが、はるかに高速なシミュレーションが可能になります。例えば、変形可能なグリッパーパッドをモデリングするための `フル <https://github.com/google-deepmind/mujoco/blob/main/model/flex/gripper.xml>`__ と `トリリニア <https://github.com/google-deepmind/mujoco/blob/main/model/flex/gripper_trilinear.xml>`__ フレックスを比較する右の動画を参照してください。


.. image:: images/computation/ccd_light.gif
      :width: 20%
      :align: right
      :class: only-light

.. image:: images/computation/ccd_dark.gif
   :width: 20%
   :align: right
   :class: only-dark

2. 3.2.3で導入され、 :ref:`nativeccd<option-flag-nativeccd>` フラグで有効化されるネイティブ凸衝突検出パイプラインがデフォルトになりました。詳細は :ref:`凸衝突検出<coCCD>` のセクションを参照してください。

   **移行方法:** 新しいパイプラインがワークフローに問題を起こす場合は、 :ref:`nativeccd<option-flag-nativeccd>` を「disable」に設定してください。

全般
^^^^^^^
3. MuJoCoビューワで ``viewport`` プロパティ、 ``set_figures`` メソッド、 ``clear_figures`` メソッドを公開することで、カスタムプロットのサポートを追加しました。
4. :ref:`flex<deformable-flex>` の衝突メッシュと変形メッシュを分離しました。これにより、高解像度衝突の忠実性を維持しながら、ソフトボディ計算のコストを固定にすることが可能になります。
5. :ref:`ポテンシャル<sensor-e_potential>` および :ref:`運動<sensor-e_kinetic>` エネルギーセンサーを追加しました。
6. ネイティブレンダラーの影レンダリングを改善しました。
7. ``introspect`` を ``python/introspect`` に移動しました。

.. admonition:: Breaking API changes
   :class: attention

   8. 上記の通り、ネイティブ凸衝突検出パイプラインがデフォルトになりました。これにより一部のワークフローが壊れる可能性があります。その場合は、 :ref:`nativeccd<option-flag-nativeccd>` を「disable」に設定して以前の動作を復元してください。
   9. :ref:`mjs_setDeepCopy` API関数を追加しました。ディープコピーフラグが0の場合、モデルのアタッチ時に親へのコピーが行われないため、子への元の参照をアタッチ後に親を変更するために使用できます。デフォルトの動作はこのようなシャローコピーを実行することです。アタッチ時に子モデルのディープコピーを作成する以前の動作は、ディープコピーフラグを1に設定することで復元できます。
   10. メッシュからの慣性推定に関する変更：

       以前は、質量が表面にあることを指定するために、任意のジオムタイプに対して :ref:`geom/shellinertia<body-geom-shellinertia>` を使用できました。現在、ジオムがメッシュの場合、この属性は無視されます。代わりに、メッシュの慣性推定は :ref:`asset/mesh/inertia<asset-mesh-inertia>` 属性を使用してアセットで指定します。

       以前は、体積慣性計算が失敗した場合（例えば非常に平坦なメッシュの場合）、コンパイラは暗黙的に表面慣性計算にフォールバックしていました。現在は、コンパイラが情報を含むエラーをスローします。
   11. コンポジットタイプ ``grid`` を削除しました。ユーザーは代わりに :ref:`flexcomp<body-flexcomp>` を使用してください。
   12. コンポジットタイプ ``particle`` を削除しました。代わりにより汎用的な :ref:`replicate<replicate>` を使用することを推奨します。例えば `このモデル <https://github.com/google-deepmind/mujoco/blob/main/model/replicate/particle.xml>`__ を参照してください。

MJX
^^^
13. 内部球体および円筒巻き付きを持つ空間テンドンのサポートを追加しました。
14. ボックス同士の衝突に関するバグを修正しました :issue:`2356` 。

Pythonバインディング
^^^^^^^^^^^^^^^^^^^^^^^

15. ``mujoco.rollout`` （マルチスレッドシミュレーションロールアウト用のPythonモジュール）の教育用colabノートブックを追加しました。こちらから利用できます |rollout_colab| 。
    |br| :github:user:`aftersomemath` による貢献です。

.. |rollout_colab| image:: https://colab.research.google.com/assets/colab-badge.png
                   :target: https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/python/rollout.ipynb

Version 3.2.7 (Jan 14, 2025)
----------------------------

Pythonバインディング
^^^^^^^^^^^^^^^^^^^^^^^
1. :ref:`rollout<PyRollout>` がネイティブマルチスレッドに対応しました。長さ ``nthread`` の ``MjData`` インスタンスのシーケンスが渡されると、 ``rollout`` は自動的にスレッドプールを作成し、計算を並列化します。スレッドプールは呼び出し間で再利用できますが、その場合は複数のスレッドから同時に関数を呼び出すことはできません。複数のスレッドロールアウトを同時に実行するには、スレッドプールをカプセル化する新しいクラス ``Rollout`` を使用してください。 :github:user:`aftersomemath` による貢献です。
2. ``mjpython`` 使用時のグローバル名前空間の汚染を修正しました（ :issue:`2265` ）。

全般
^^^^^^^

.. admonition:: Breaking API changes (minor)
   :class: attention

   3. ``mjData.qLDiagSqrtInv`` フィールドが削除されました。このフィールドは双対ソルバーにのみ必要です。無条件に計算するのではなく、必要に応じて計算されるようになりました。関連して、 :ref:`mj_solveM2` に対応する引数を追加しました。

4. PGSソルバーの :ref:`A行列<soDual>` のメモリフットプリントを削減しました。これはMuJoCoに残っていた最後の密メモリ割り当てであり、 :ref:`動的メモリ割り当てヒューリスティック<CSize>` の大幅な削減を可能にしました。

バグ修正
^^^^^^^^^
5. ボックス・球体コライダーのバグを修正しました。深い貫通時に深度が不正確でした（ :issue:`2206` ）。
6. :ref:`mj_mulM2` のバグを修正し、テストを追加しました。

Version 3.2.6 (Dec 2, 2024)
---------------------------

全般
^^^^^^^
1. :ref:`composite<body-composite>` から rope と loop を削除しました。ユーザーはそれぞれ :at:`cable` プラグインまたは :ref:`flexcomp<body-flexcomp>` を代わりに使用することを推奨します。

MJX
^^^
2. 筋肉アクチュエータを追加しました。

Pythonバインディング
^^^^^^^^^^^^^^^^^^^^^^^
3. Python 3.13用のビルド済みホイールを提供するようにしました。
4. :ref:`mjSpec` オブジェクトに ``bind`` メソッドを追加し、id属性を削除しました。アタッチとデタッチを繰り返すシナリオではidの使用はエラーが発生しやすいです。Pythonユーザーはモデル要素の一意な識別に名前を使用することを推奨します。
5. :ref:`rollout<PyRollout>` が長さ ``nroll`` の MjModel のシーケンスを受け付けるようになりました。また、値が常に推測可能なため ``nroll`` 引数を削除しました。

バグ修正
^^^^^^^^^
6. :issue:`2212` を修正しました。 ``mjx.get_data`` の型エラーです。
7. 3.2.0で導入された :ref:`texrepeat<asset-material-texrepeat>` 属性の処理に関するバグを修正しました。誤って ``float`` から ``int`` にキャストされていました（ :issue:`2223` を修正）。

Version 3.2.5 (Nov 4, 2024)
---------------------------

機能の昇格
^^^^^^^^^^^^^^^^^
1. 3.2.0で開発中の機能として導入された :ref:`mjSpec` による :doc:`モデル編集<programming/modeledit>` フレームワークが安定版となり、一般利用が推奨されるようになりました。
2. 3.2.3で導入され、 :ref:`nativeccd<option-flag-nativeccd>` フラグで有効化されるネイティブ凸衝突検出パイプラインは、まだデフォルトではありませんが、すでに一般利用が推奨されています。衝突関連の問題が発生した場合はお試しいただき、問題があればご報告ください。

全般
^^^^^^^

3. グローバルコンパイラフラグ ``exactmeshinertia`` が削除され、メッシュ固有の :ref:`inertia<asset-mesh-inertia>` 属性に置き換えられました。
4. 有用でなかった ``convexhull`` コンパイラオプション（メッシュの凸包計算を無効にするオプション）が削除されました。
5. 非推奨の ``mju_rotVecMat`` 、 ``mju_rotVecMatT`` および ``mjv_makeConnector`` 関数を削除しました。
6. ソートがより高速なネイティブソート関数を使用するようになりました（ :issue:`1638` を修正）。
7. 3.2.1で導入されたPBRテクスチャレイヤーが、個別のサブ要素から単一の :ref:`layer<material-layer>` サブ要素にリファクタリングされました。
8. コンポジットタイプの box、cylinder、sphere が削除されました。ユーザーは代わりに :ref:`flexcomp<body-flexcomp>` で利用可能な同等のタイプを使用してください。

MJX
^^^
9. ``apply_ft`` 、 ``jac`` 、および ``xfrc_accumulate`` をパブリック関数として追加しました。
10. ``TOUCH`` センサーを追加しました。
11. ``eq_active`` のサポートを追加しました。 :issue:`2173` を修正。
12. 楕円体とのレイ交差を追加しました。

バグ修正
^^^^^^^^^
13. サイトセマンティクスを持つ接続制約および溶接制約に関連する複数のバグを修正しました（ :issue:`2179` を修正、 :github:user:`yinfanyi` による報告）。3.2.3で接続制約と溶接制約にサイト指定が導入されたことで、 `mjData.eq_obj1id` と `mjData.eq_obj2id` のセマンティクスが条件付きで変更されましたが、これらの変更がいくつかの箇所で正しく伝播されておらず、制約慣性の不正な計算、影響を受ける力/トルクセンサーの読み取り値の誤り、およびそのような制約の実行時の有効化/無効化に問題が生じていました。
14. スライダークランク :ref:`伝達<geTransmission>` のバグを修正しました。このバグは3.0.0で導入されたものです。
15. フレックスのテクスチャ座標のバグを修正しました。mjModelでのテクスチャの正しい割り当てが妨げられていました。


ドキュメント
^^^^^^^^^^^^^
16. :doc:`APIリファレンス <../APIreference/APIfunctions>` の関数ヘッダーがGitHub上のソース定義にリンクされるようになりました。

Version 3.2.4 (Oct 15, 2024)
----------------------------

全般
^^^^^^^

.. youtube:: e8lUuykQPGs
   :aspect: 16:7
   :align: right
   :width: 240px

1. Newtonソルバーが ``nv*nv`` のメモリ割り当てを必要としなくなり、はるかに大きなモデルが可能になりました。例えば `100_humanoids.xml  <https://github.com/google-deepmind/mujoco/blob/main/model/humanoid/100_humanoids.xml>`__ を参照してください。完全なスパース化のためにまだ2つの二次メモリ割り当てが残っています: ``mjData.actuator_moment`` およびPGSソルバーが使用する行列です。
2. :at:`solid` および :at:`membrane` プラグインを削除し、関連する計算をエンジンに移動しました。以前これらのプラグインを必要としていたフレックスオブジェクトの例として、 `3Dサンプルモデル <https://github.com/google-deepmind/mujoco/blob/main/model/flex/floppy.xml>`__ および `2Dサンプルモデル <https://github.com/google-deepmind/mujoco/blob/main/model/flex/trampoline.xml>`__ を参照してください。
3. 関数 ``mjs_setActivePlugins`` を :ref:`mjs_activatePlugin` に置き換えました。

MJX
^^^
4. キネマティクスに ``mocap_pos`` と ``mocap_quat`` を追加しました。
5. プーリーおよび外部球体・円筒巻き付きを持つ :ref:`空間テンドン <tendon-spatial>` のサポートを追加しました。
6. 球体-円筒および球体-楕円体の衝突関数を追加しました（ :issue:`2126` ）。
7. 摩擦損失制約のバグを修正しました。
8. ``TENDONPOS`` および ``TENDONVEL`` センサーを追加しました。
9. ``_decode_pyramid`` における接線方向の接触力の計算に関するバグを修正しました。
10. ``JOINTINPARENT`` アクチュエータ伝達タイプを追加しました。

Pythonバインディング
^^^^^^^^^^^^^^^^^^^^^^^
11. Python 3.8のサポートを削除しました。 `上流で非推奨 <https://devguide.python.org/versions>`__ となったためです。

バグ修正
^^^^^^^^^
12. MJXで ``actuator_force`` が設定されていなかったバグを修正しました（ :issue:`2068` ）。
13. ``mjx.put_data`` を呼び出した後にMJXのデータのテンドンフィールドが不正になるバグを修正しました。
14. ハイトフィールドが :ref:`衝突センサー<collision-sensors>` と共に使用された場合に、まだサポートされていないためコンパイラがエラーを返すようになりました。


Version 3.2.3 (Sep 16, 2024)
----------------------------

全般
^^^^^^^

.. admonition:: Breaking API changes
   :class: attention

   1. 実行時オプション ``mpr_tolerance`` および ``mpr_iterations`` が、XMLおよび :ref:`mjOption` 構造体の両方で :ref:`ccd_tolerance<option-ccd_tolerance>` および :ref:`ccd_iterations<option-ccd_iterations>` に名前が変更されました。これは、新しい凸衝突検出パイプライン（下記参照）がMPRアルゴリズムを使用しないためです。これらのオプションのセマンティクスは変更されていません。
   2. 関数 ``mjs_findMesh`` と ``mjs_findKeyframe`` が ``mjs_findElement`` に置き換えられ、任意のオブジェクトタイプの検索が可能になりました。
   3. :ref:`composite<body-composite>` での2D/3D弾性プラグインの実験的使用が削除されました。ユーザーは代わりに :ref:`flexcomp<body-flexcomp>` を使用してください。こちらは正しい衝突動作を提供します。

4. :ref:`nativeccd<option-flag-nativeccd>` フラグを追加しました。このフラグが有効な場合、一般的な凸衝突検出は `libccd <https://github.com/danfis/libccd>`__ ではなく新しいネイティブコードパスで処理されます。この機能はテストの初期段階にありますが、衝突検出に関連する問題を経験したユーザーは試用して問題を報告してください。

.. youtube:: kcM_oauk3ZA
   :aspect: 16:7
   :align: right
   :width: 240px

5. 2つのサイトを使用して :ref:`connect<equality-connect>` および :ref:`weld<equality-weld>` 等式制約を定義する新しい方法を追加しました。この新しいセマンティクスは、制約が基準構成で満たされるという仮定が成り立たない場合に便利です。この場合、シミュレーションの開始時にサイトが「スナップ」して結合します。さらに、実行時にサイトの位置（ ``mjModel.site_pos`` ）と向き（ ``mjModel.site_quat`` ）を変更すると、制約定義が正しく変更されます。新しいセマンティクスを使用した `サンプルモデル <https://github.com/google-deepmind/mujoco/blob/main/test/engine/testdata/equality_site.xml>`__ が右の動画に示されています。
6. **フリージョイントのアライメント** を導入しました。これはフリージョイントを持ち子ボディを持たないボディ（単純な自由浮遊ボディ）に適用される最適化で、ボディフレームを慣性フレームに自動的にアライメントします。この機能は :ref:`freejoint/align<body-freejoint-align>` 属性で個別に、またはコンパイラの :ref:`alignfree<compiler-alignfree>` 属性でグローバルに切り替えることができます。アライメントにより関連する6x6慣性サブ行列が対角化され、自由ボディのシミュレーションがより高速かつ安定になります。

   この最適化は厳密な改善ですが、ジョイントの自由度のセマンティクスが変更されます。そのため、古いバージョンで保存された ``qpos`` および ``qvel`` の値（例えば :ref:`キーフレーム<keyframe>` 内のもの）は無効になります。この潜在的な破壊のため、グローバルコンパイラ属性は現在デフォルトで「false」ですが、将来のリリースで「true」に変更される可能性があります。すべての新しいモデルにはアライメントされたフリージョイントが推奨されます。

7. バッファから直接テクスチャを作成するための :ref:`mjSpec` オプションを追加しました。
8. :ref:`シェル（表面）慣性 <body-geom-shellinertia>` がすべてのジオムタイプでサポートされるようになりました。
9. サブモデルの :ref:`アタッチメント<meAttachment>` 時に、 :ref:`キーフレーム<keyframe>` が親モデルに正しくマージされるようになりました。ただし、最初のアタッチメント時のみです。
10. :ref:`mjtSameFrame` 列挙型を追加しました。ボディとその子の可能なフレームアライメントが含まれます。これらのアライメントは :ref:`mj_kinematics` での計算ショートカットに使用されます。
11. 運動学的ヤコビアンの時間導関数を計算するための :ref:`mj_jacDot` を追加しました。 :issue:`411` を修正。

MJX
^^^
12. ``mjx.Data`` に ``efc_pos`` を追加しました（ :issue:`1388` ）。
13. 位置依存センサーを追加しました: ``MAGNETOMETER`` 、 ``CAMPROJECTION`` 、 ``RANGEFINDER`` 、 ``JOINTPOS`` 、 ``ACTUATORPOS`` 、 ``BALLQUAT`` 、 ``FRAMEPOS`` 、 ``FRAMEXAXIS`` 、 ``FRAMEYAXIS`` 、 ``FRAMEZAXIS`` 、 ``FRAMEQUAT`` 、 ``SUBTREECOM`` 、 ``CLOCK`` 。
14. 速度依存センサーを追加しました: ``VELOCIMETER`` 、 ``GYRO`` 、 ``JOINTVEL`` 、 ``ACTUATORVEL`` 、 ``BALLANGVEL`` 、 ``FRAMELINVEL`` 、 ``FRAMEANGVEL`` 、 ``SUBTREELINVEL`` 、 ``SUBTREEANGMOM`` 。
15. 加速度/力依存センサーを追加しました: ``ACCELEROMETER`` 、 ``FORCE`` 、 ``TORQUE`` 、 ``ACTUATORFRC`` 、 ``JOINTACTFRC`` 、 ``FRAMELINACC`` 、 ``FRAMEANGACC`` 。
16. 未使用の（MuJoCoのみの）配列をデバイスに配置しないようにデフォルトポリシーを変更しました。
17. ``mjx.put_model`` および ``mjx.put_data`` と同等にするため、 ``mjx.make_data`` に ``device`` パラメータを追加しました。
18. :doc:`流体抵抗 <computation/fluid>` を除くすべてのケースで :ref:`implicitfast 積分<geIntegration>` のサポートを追加しました。
19. スパース質量行列に対して ``qLDiagInv`` のサイズが間違っていたバグを修正しました。
20. ジョイントおよびテンドンの :ref:`摩擦損失 <coFriction>` のサポートを追加しました。
21. 2つのサイトを使用する :ref:`connect<equality-connect>` 等式制約のサポートを追加しました。
22. サイト巻き付きを持つ :ref:`空間テンドン <tendon-spatial>` のサポートを追加しました。

バグ修正
^^^^^^^^^
23. 3.1.7で導入されたメッシュのバウンディングボリューム階層のパフォーマンス低下を修正しました（ :issue:`1875` 、 :github:user:`michael-ahn` による貢献）。
24. 筋肉とステートレスアクチュエータの両方を持ち、陰的積分器のいずれかを使用するモデルで、誤った導関数が計算されるバグを修正しました。
25. 球体周りのテンドン巻き付きのバグを修正しました。この修正前は、外部に配置された :ref:`sidesite<spatial-geom-sidesite>` を持つ球体に巻き付くテンドンが、球体を巻き回る代わりに内部に入り込む可能性がありました。
26. モデルの :ref:`アタッチメント<meAttachment>` 時に :at:`meshdir` と :at:`texturedir` が上書きされ、異なるディレクトリにアセットを持つモデルのアタッチメントが妨げられるバグを修正しました。

Pythonバインディング
^^^^^^^^^^^^^^^^^^^^^^^
27. :ref:`mjSpec` でのエンジンプラグインのサポートを追加しました（ :issue:`1903` ）。
28. モデルの読み込み時のアセットディクショナリに関する問題のエラー報告を改善しました。


Version 3.2.2 (Aug 8, 2024)
---------------------------

全般
^^^^^^^
1. テクスチャとマテリアルの制限を1000に戻しました。3.2.0で誤ってこの制限が100に減少し、一部の既存モデルが壊れていました（ :issue:`1877` ）。

Version 3.2.1 (Aug 5, 2024)
---------------------------

全般
^^^^^^^
1. ``mjModel.tex_rgb`` を ``mjModel.tex_data`` に名前変更しました。
2. NaNまたは無限大が検出された際の自動リセットを無効にするための新しい :ref:`autoreset<option-flag-autoreset>` フラグを追加しました。
3. 複数のテクスチャをレンダリング用に指定できるように、MJCFの :ref:`material<asset-material>` 要素にサブ要素を追加しました（例: ``occlusion, roughness, metallic`` ）。MuJoCoレンダラーはこれらの新機能をサポートしておらず、外部レンダラーでの使用のために提供されています。
4. ソート（ ``mjQUICKSORT`` ）がC++でビルドする際に ``std::sort`` を呼び出すようになりました（ :issue:`1638` ）。

MJX
^^^
5. 対応するMuJoCo構造体とのさらなる互換性のため、 ``mjx.Model`` と ``mjx.Data`` にフィールドを追加しました。
6. :ref:`固定テンドン <tendon-fixed>` のサポートを追加しました。
7. テンドン長さリミットのサポートを追加しました（ :ref:`mjtConstraint` の ``mjCNSTR_LIMIT_TENDON`` ）。
8. テンドン等式制約のサポートを追加しました（ :ref:`mjtEq` の ``mjEQ_TENDON`` ）。
9. テンドンアクチュエータ伝達のサポートを追加しました（ :ref:`mjtTrn` の ``mjTRN_TENDON`` ）。

Pythonバインディング
^^^^^^^^^^^^^^^^^^^^^^^
10. ``mujoco.spec.from_file`` 、 ``mujoco.spec.from_string`` および ``mujoco.spec.compile`` でアセットディクショナリ引数のサポートを追加しました。


バグ修正
^^^^^^^^^
11. 陰的積分器が無効化されたアクチュエータを考慮していなかったバグを修正しました（ :issue:`1838` ）。

Version 3.2.0 (Jul 15, 2024)
----------------------------

新機能
^^^^^^^^^^^^

1. 主要な新機能として **プロシージャルモデル作成・編集** を導入しました。新しいトップレベルデータ構造
   :ref:`mjSpec` を使用します。詳細は :doc:`モデル編集<programming/modeledit>` の章を参照してください。
   このリリース時点では、この機能はまだテスト中であり、将来の破壊的変更の対象となる可能性があります。
   :issue:`364` を修正。

全般
^^^^^^^

.. admonition:: Breaking API changes
   :class: attention

   2. 非推奨の ``mj_makeEmptyFileVFS`` および ``mj_findFileVFS`` 関数を削除しました。定数 ``mjMAXVFS`` および
      ``mjMAXVFSNAME`` も不要になったため削除されました。

      **移行方法:** :ref:`mj_addBufferVFS` を使用してバッファをVFSファイルに直接コピーしてください。

   3. :ref:`mj_defaultVFS` の呼び出しはVFS内部でメモリを割り当てる場合があり、内部で割り当てられたメモリを解放するために
      対応する :ref:`mj_deleteVFS` を呼び出す必要があります。

   4. ``mju_rotVecMat`` および ``mju_rotVecMatT`` を非推奨とし、 :ref:`mju_mulMatVec3` および
      :ref:`mju_mulMatTVec3` に置き換えました。これらの新しい関数名と引数の順序はAPIの他の部分とより一貫性があります。
      旧関数はPythonバインディングから削除済みで、次のリリースでC APIからも削除されます。
   5. アクチュエータプラグインから ``actuator_actdim`` コールバックを削除しました。アクチュエータプラグインは ``actdim`` 属性を
      持つようになり、 ``act`` 配列に状態を書き込むアクチュエータではこの属性を使用する必要があります。これにより、ステートフルな
      アクチュエータプラグインを持つモデルでキーフレームを使用した際に発生していたクラッシュが修正されました。PIDプラグインは
      actdimの値が正しくない場合にエラーを返すようになりました。

6. MJCFに :ref:`attach<body-attach>` メタ要素を追加しました。これにより、異なるモデルのサブツリーを現在のモデルのボディに
   :ref:`アタッチ<meAttachment>` できるようになりました。
7. :ref:`VFS<Virtualfilesystem>` の実装がC++で書き直され、速度とメモリフットプリントの両面で大幅に効率化されました。

.. youtube:: ZXBTEIDWHhs
   :aspect: 16:7
   :align: right
   :width: 240px

8. 正射影カメラのサポートを追加しました。固定カメラとフリーカメラの両方で利用可能で、それぞれ
   ``camera/orthographic`` および :ref:`global/orthographic<visual-global-orthographic>`
   属性を使用します。
9. メッシュの凸包の最大頂点数を指定する :ref:`maxhullvert<asset-mesh-maxhullvert>` を追加しました。
10. 現在の状態をモデルのキーフレームに保存するための :ref:`mj_setKeyframe` を追加しました。
11. URDFパーサーで ``ball`` ジョイント（URDFでは「spherical」）のサポートを追加しました。
12. 以前は `mjtnum.h <https://github.com/google-deepmind/mujoco/blob/main/include/mujoco/mjtnum.h>`__
    にハードコードされていた ``mjUSEDOUBLE`` を、ビルド時フラグ ``mjUSESINGLE`` に置き換えました。このシンボルが
    定義されていない場合、MuJoCoは通常通り倍精度浮動小数点を使用します。 ``mjUSESINGLE`` が定義されている場合、
    MuJoCoは単精度浮動小数点を使用します。 :ref:`mjtNum` を参照してください。

    関連して、単精度でのビルドを妨げていた様々な型エラーを修正しました。
13. ``mjData.qpos`` および ``mjData.mocap_quat`` 内のクォータニオンが :ref:`mj_kinematics` によって
    インプレースで正規化されなくなりました。代わりに、使用時に正規化されます。最初のステップの後、
    ``mjData.qpos`` 内のクォータニオンは正規化されます。
14. コンパイラでのメッシュ読み込み（通常、読み込みプロセスの最も遅い部分）がマルチスレッド化されました。

MJX
~~~
15. :ref:`楕円摩擦錐<option-cone>` のサポートを追加しました。
16. 一部の困難な制約設定で最適でない直線探索解が得られるバグを修正しました。
17. Newtonソルバーで最適でない勾配が得られることがあるバグを修正しました。


.. youtube:: P83tKA1iz2Y
   :align: right
   :width: 360px

Simulate
^^^^^^^^
18. 改善されたチュートリアル動画を追加しました。
19. ブラウン運動ノイズ生成器を改善しました。
20. モデルの読み込み時間が0.25秒を超える場合に表示するようにしました。

Pythonバインディング
^^^^^^^^^^^^^^^^^^^^^^^
21. ``mujoco.MjData`` インスタンスに対して ``copy.deepcopy()`` を使用した際のメモリリークを修正しました（ :issue:`1572` ）。

バグ修正
^^^^^^^^^
22. ``mj_copyData`` （Pythonバインディングでは ``copy.copy()`` ）が接触情報を正しくコピーしていなかった問題を修正しました
    （ :issue:`1710` ）。
23. XMLへの保存時にフレームが複数回書き込まれる問題を修正しました（ :issue:`1802` ）。

Version 3.1.6 (Jun 3, 2024)
---------------------------

全般
^^^^^^^

1. 2つのジオム間の最短符号付き距離と、オプションでそれらを結ぶ線分を計算するための :ref:`mj_geomDistance` を追加しました。
   関連して、3つのセンサーを追加しました: :ref:`distance<sensor-distance>` 、 :ref:`normal<sensor-normal>` 、
   :ref:`fromto<sensor-fromto>` 。詳細は関数およびセンサーのドキュメントを参照してください。 :issue:`51` を修正。
2. 位置アクチュエータの改善:

   - :ref:`position actuator<actuator-position>` に :ref:`timeconst<actuator-position-timeconst>` 属性を追加しました。
     正の値に設定すると、アクチュエータは :at:`filterexact` ダイナミクスを持つステートフルなアクチュエータになります。
   - :el:`position` および :el:`intvelocity` アクチュエータの両方に :ref:`dampratio<actuator-position-dampratio>` を追加しました。
     :at:`kv` 属性の代替として、自然な単位を使用してアクチュエータの減衰を設定する便利な方法を提供します。
     詳細は属性のドキュメントを参照してください。

MJX
^^^

3. ハイトフィールドの衝突サポートを追加しました。 :issue:`1491` を修正。
4. メッシュプロパティをvmapできるように、事前コンパイル済みフィールド ``mesh_convex`` を ``mjx.Model`` に追加しました。
   :issue:`1655` を修正。
5. 凸メッシュ衝突で、面分離軸が見つかっているにもかかわらず誤ったエッジ接触が生成されるバグを修正しました。
   :issue:`1695` を修正。

バグ修正
^^^^^^^^^

6. :ref:`fusestatic<compiler-fusestatic>` が有効な場合（URDFインポートでよくあるケース）に衝突が見逃される可能性がある
   バグを修正しました。 :issue:`1069` 、 :issue:`1577` を修正。
7. SDF反復の可視化が、格納用ベクトルのサイズ外に書き込むバグを修正しました。 :issue:`1539` を修正。

Version 3.1.5 (May 7, 2024)
---------------------------

全般
^^^^^^^

.. youtube:: 5k0_wsIRAFc
   :aspect: 16:7
   :align: right
   :width: 240px

1. MJCFに :ref:`replicate<replicate>` を追加しました。増分的な並進および回転オフセットでサブツリーを繰り返すことを可能にする
   :ref:`メタ要素<meta-element>` です。
2. MuJoCoコンパイラの内部キャッシュを有効化し、再コンパイルを高速化しました。現在、処理済みの
   テクスチャ、hfield、OBJメッシュがキャッシュされます。Unity環境のサポートはまだ利用できません。
3. ``mjModel.mesh_scale`` を追加しました: :ref:`scale<asset-mesh-scale>` 属性で指定された、アセット頂点に適用される
   スケーリングです。
4. ネイティブレンダラーでは無視されるが、外部レンダラーで使用可能なビジュアルプロパティを追加しました:

   - :ref:`light/bulbradius<body-light-bulbradius>` 属性と対応する ``mjModel.light_bulbradius`` フィールド。
   - :ref:`material/metallic<asset-material-metallic>` 属性と対応する ``mjModel.material_metallic`` フィールド。
   - :ref:`material/roughness<asset-material-roughness>` 属性と対応する ``mjModel.material_roughness``
     フィールド。
5. :ref:`mj_stackAllocNum` および :ref:`mj_stackAllocInt` の ``size`` 引数の型が ``int`` から
   ``size_t`` に変更されました。
6. :ref:`flexcomp<body-flexcomp-file>` でgmshフォーマットバージョン2.2のサーフェスメッシュのサポートを追加しました。

MJX
^^^
.. admonition:: Breaking API changes
   :class: attention

   7. 重要な新機能が欠如していたため、非推奨の ``mjx.device_get_into`` および ``mjx.device_put`` 関数を削除しました。

      **移行方法:** ``mjx.device_get_into`` の代わりに ``mjx.get_data_into`` を、 ``mjx.device_put`` の代わりに
      ``mjx.put_data`` を使用してください。

8. 円筒と平面の衝突を追加しました。
9. ``mjx.Data`` に ``efc_type`` を、 ``mjx.Contact`` に ``dim`` と ``efc_address`` を追加しました。
10. ``mjx.Contact`` に ``geom`` を追加し、 ``geom1`` 、 ``geom2`` を非推奨としました。
11. ``mujoco.MjData`` と一致させるため、 ``mjx.Data`` に ``ne`` 、 ``nf`` 、 ``nl`` 、 ``nefc`` 、 ``ncon`` を追加しました。
12. 上記のフィールド追加に伴い、 ``mjx.get_params`` 、 ``mjx.ncon`` 、 ``mjx.count_constraints`` を削除しました。
13. メッシュが多数のジオムで複製されている場合の衝突検出を高速化するため、デバイス上でのメッシュの構成方法を変更しました。
14. ブロードフェーズ衝突チェックでカプセルが無視される可能性があるバグを修正しました。
15. SDFを使用した円筒の衝突を追加しました。
16. すべての :ref:`condim <coContact>` のサポートを追加しました: 1, 3, 4, 6。
17. :ref:`mj_id2name` と :ref:`mj_name2id` のMJX版である ``id2name`` と ``name2id`` のサポート関数を追加しました。
18. :ref:`gravcomp<body-gravcomp>` および :ref:`actuatorgravcomp<body-joint-actuatorgravcomp>` のサポートを追加しました。
19. ``mjx.ray`` でレイ-メッシュテストに対して負の距離が許可されることがあるバグを修正しました。
20. MJX物理ステップから自動的に導出された解析的勾配を使用して移動ポリシーを学習する、新しい `微分可能物理チュートリアル <https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/mjx/training_apg.ipynb>`__ を追加しました。
    :github:user:`Andrew-Luo1` による貢献です。

バグ修正
^^^^^^^^^
21. ライトのデフォルトが保存されていなかった問題を修正しました。
22. XMLの保存時にボディ名によってフレーム名が上書きされる問題を修正しました。3.1.4で導入されたバグです。
23. :ref:`mj_saveModel` のPythonバインディングのバグを修正しました: ``buffer`` 引数はオプションとドキュメントに記載されていましたが、
    実際にはオプションではありませんでした。
24. 2.15GBを超えるメモリ割り当てが妨げられるバグを修正しました。 :issue:`1606` を修正。


Version 3.1.4 (April 10th, 2024)
--------------------------------

全般
^^^^^^^
.. admonition:: Breaking API changes
   :class: attention

   1. センサーにネイティブにノイズを追加する機能を削除しました。 ``mjModel.sensor_noise`` フィールドと
      :ref:`対応する属性<CSensor>` は保持され、ユーザーが独自の使用のために標準偏差情報を保存する便利な場所として
      機能するようになりました。この機能は以下の理由で削除されました:

      - ランダムノイズ生成器をシードするメカニズムがなかった。
      - スレッドセーフではなく、シード機能が提供されたとしても、複数スレッドでのサンプリングは再現不可能な結果を
        もたらす。
      - この機能はエンジンの越権行為と見なされた。ノイズの追加はユーザーの責任であるべきである。
      - この機能を実際に使用している人がいないと認識されていた。

      **移行方法:** センサー値へのノイズ追加はユーザー自身で行ってください。

2. ジョイント属性 :ref:`actuatorgravcomp<body-joint-actuatorgravcomp>` を追加しました。有効にすると、ジョイントに対する
   重力補償力がアクチュエータによって適用されたものとして扱われます。詳細は属性のドキュメントを参照してください。サンプルモデル
   `refsite.xml <https://github.com/google-deepmind/mujoco/blob/main/test/engine/testdata/actuation/refsite.xml>`__
   はアームのデカルトアクチュエーションを示しており、この属性を使用するよう更新されました。
3. 例えば `fTetwild <https://github.com/wildmeshing/fTetWild>`__ で生成されるgmshフォーマット2.2の四面体メッシュの
   サポートを追加しました。

4. オイラー角列をクォータニオンに変換するための :ref:`mju_euler2Quat` を追加しました。

MJX
^^^
5. 凸衝突のSATのパフォーマンスを改善しました。
6. 球体/カプセルと凸体の深い貫通のバグを修正しました。
7. ``mjx.put_data`` で生成された ``mjx.Data`` が ``mjx.make_data`` と異なるtreedefを持つバグを修正しました。
8. 凸メッシュ衝突でmargin/gapがサポートされていないため、これらが指定された場合にエラーをスローするようにしました。
9. 楕円体と平面の衝突を追加しました。
10. userdataのサポートを追加しました。
11. 符号付き距離関数（SDF）を使用した楕円体同士および楕円体-カプセルの衝突を追加しました。

Simulate
^^^^^^^^
12. 有効化フラグ文字列の順序のバグを修正しました。この変更前は、simulate UIを使用して
    :ref:`invdiscrete<option-flag-invdiscrete>` フラグまたは（現在は削除された） ``sensornoise`` フラグを
    トグルすると、実際には別のフラグがトグルされていました。

Pythonバインディング
^^^^^^^^^^^^^^^^^^^^^^^

.. youtube:: xHDS0n5DpqM
   :aspect: 16:7
   :align: right
   :width: 240px

13. 非線形最小二乗法のための ``mujoco.minimize`` Pythonモジュールを追加しました。システム同定（sysID）向けに設計されています。
    sysIDチュートリアルは作成中ですが、逆運動学を含むサンプル付きの教育用colabノートブックがこちらで利用可能です: |ls_colab|
    |br| 右の動画はチュートリアルのサンプルクリップです。

.. |ls_colab| image:: https://colab.research.google.com/assets/colab-badge.png
              :target: https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/python/least_squares.ipynb

Version 3.1.3 (March 5th, 2024)
-------------------------------

全般
^^^^^^^
1. :ref:`position<actuator-position>` および :ref:`intvelocity<actuator-intvelocity>` アクチュエータに
   :at:`inheritrange` 属性を追加しました。伝達対象（ジョイントまたはテンドン）の範囲に応じて、アクチュエータの
   :at:`ctrlrange` または :at:`actrange` （それぞれ）を便利に設定できます。詳細は
   :ref:`position/inheritrange<actuator-position-inheritrange>` を参照してください。
2. ``mj_makeEmptyFileVFS`` を非推奨とし、 :ref:`mj_addBufferVFS` を代替としました。 :ref:`mjVFS` は内部ファイルバッファの
   チェックサムを計算するようになりました。 :ref:`mj_addBufferVFS` は指定された名前のバッファをmjVFSに割り当て、データバッファを
   コピーします。これにより、非推奨となった ``mj_makeEmptyFileVFS`` を呼び出した後にmjVFS内部ファイルバッファに直接コピーする
   という2段階のプロセスが統合・置換されます。
3. 一般化速度からサブツリー角運動量への線形写像 :math:`h = H \dot q` を提供する ``3 x nv`` の角運動量行列 :math:`H(q)` を
   計算する :ref:`mj_angmomMat` を追加しました。 :github:user:`v-r-a` による貢献です。

MJX
^^^
4. デバイスデータの取得・送信のパフォーマンスを改善しました。

   - numpy配列のシリアライズに ``tobytes()`` を使用するようにしました。タプルへの変換と比べて桁違いに高速です。
   - 配列の形状が変更されていない場合、ホスト側の ``mjData`` 配列の再割り当てを回避するようにしました。
   - 多数のジオムを持つモデルに対する ``mjx.ncon`` の計算を高速化しました。
   - ``nc`` が ``mjx.Data`` から導出可能な場合、 ``mjx.get_data_into`` 内での ``mjx.ncon`` 呼び出しを回避するようにしました。
5. ``mjx-viewer`` の実行を妨げていたバグを修正しました。新しい ``mjx.get_data_into`` 関数呼び出しを使用するように
   ``mjx-viewer`` を更新しました。
6. 密な質量行列を使用する際に不正な減衰が適用されていた ``mjx.euler`` のバグを修正しました。
7. :ref:`mjtSolver` で ``mjSOL_NEWTON`` を使用する際に収束が遅くなっていた ``mjx.solve`` のバグを修正しました。
8. ``mjx.Model`` に :ref:`mjOption.impratio<mjOption>` のサポートを追加しました。
9. ``mjx.Model`` および ``mjx.Data`` にカメラのサポートを追加しました。 :issue:`1422` を修正。
10. `top_k` とバウンディング球を使用したブロードフェーズの実装を追加しました。

Pythonバインディング
^^^^^^^^^^^^^^^^^^^^^^^
11. ``mjContact`` 構造体の ``geom`` 、 ``vert`` 、 ``elem`` 、 ``flex`` 配列メンバー、および ``mjrContext`` 構造体の
    全配列メンバーのバインディングにおける不正なデータ型を修正しました。

Version 3.1.2 (February 05, 2024)
---------------------------------

全般
^^^^^^^
1. :ref:`discardvisual<compiler-discardvisual>` コンパイラフラグを改善しました。すべてのビジュアル専用アセットが破棄される
   ようになりました。詳細は :ref:`discardvisual<compiler-discardvisual>` を参照してください。
2. ミッドフェーズ衝突検出の :ref:`タイマー<mjtTimer>` を削除し、ナローフェーズタイマーに統合しました。これは、2つのフェーズを
   別々に計測するには衝突関数内部できめ細かいタイマーが必要だったためです。これらの関数は非常に小さく高速であるため、タイマー自体が
   計測可能なコストを発生させていました。
3. ``visual/global`` にフラグ :ref:`bvactive<visual-global-bvactive>` を追加しました。アクティブなバウンディングボリュームの
   可視化（ :ref:`この変更履歴項目<midphase>` の赤/緑のボックス）をオフにできます。非常に高解像度のメッシュを持つモデルでは、
   この可視化に必要な計算がシミュレーション速度を低下させる可能性があります。 :issue:`1279` を修正。

   - :ref:`バウンディングボリューム<visual-rgba-bv>` および :ref:`アクティブバウンディングボリューム<visual-rgba-bvactive>`
     の色を :ref:`visual/rgba<visual-rgba>` に追加しました。
4. ハイトフィールドの標高データを :ref:`elevation<asset-hfield-elevation>` 属性でXML内に直接指定できるようになりました
   （PNGファイルだけでなく）。
   `サンプルモデル <https://github.com/google-deepmind/mujoco/blob/main/test/user/testdata/hfield_xml.xml>`__ を参照してください。

MJX
^^^
5. :ref:`dyntype<actuator-general-dyntype>` ``filterexact`` を追加しました。
6. :at:`site` 伝達を追加しました。
7. より安定した四足歩行環境を使用するようにMJX colabチュートリアルを更新しました。
8. 平面、球体、カプセル、ボックス、メッシュに対して :ref:`mj_ray` をミラーリングする ``mjx.ray`` を追加しました。
9. :ref:`mj_isSparse` をミラーリングする ``mjx.is_sparse`` と :ref:`mj_fullM` をミラーリングする ``mjx.full_m`` を
   追加しました。
10. :ref:`option-jacobian` によるスパースまたは密な質量行列の指定のサポートを追加しました。
11. ゼロでない摩擦損失が存在する場合に未実装エラーを発生させるようにしました。 :issue:`1344` を修正。

Pythonバインディング
^^^^^^^^^^^^^^^^^^^^^^^
12. :ref:`rollout<PyRollout>` モジュールの実装を改善しました。以下の変更は破壊的変更であり、依存するコードの修正が必要です。

    - 状態仕様として :ref:`mjSTATE_FULLPHYSICS<siFullPhysics>` を使用し、時間の検査による発散検出を可能にしました。
    - :ref:`ユーザー入力<siInput>` フィールドの任意の組み合わせをコントロールとして使用するユーザー定義のコントロール仕様を
      許可するようにしました。
    - 出力はスクイーズされなくなり、常にdim=3になります。
13. :ref:`パッシブビューア<PyViewerPassive>` の ``sync`` 関数が、 ``user_scn`` のレンダリングフラグの変更を反映できるように
    なりました。 :issue:`1190` でのリクエストに対応。

バグ修正
^^^^^^^^^
14. フレックスがworldbodyにない場合にプラグインでピンを使用できなかったバグを修正しました。 :issue:`1270` を修正。
15. 長さ範囲の下限の外側でゼロ以外の値が生じていた :ref:`筋肉モデル<CMuscle>` のバグを修正しました。 :issue:`1342` を修正。


Version 3.1.1 (December 18, 2023)
-----------------------------------

バグ修正
^^^^^^^^^
1. ボックス同士の衝突で、一方のボックスがもう一方に深く埋め込まれている場合に接触が生成されなかったバグを修正しました
   （3.1.0で導入）。
2. :ref:`simulate<saSimulate>` で「LOADING...」メッセージが正しく表示されなかったバグを修正しました。
3. フレックスオブジェクトを含むモデルで使用した際の、Python :ref:`パッシブビューア<PyViewerPassive>` のクラッシュを
   修正しました。
4. MJXで ``site_xmat`` が ``get_data`` および ``put_data`` で無視されていたバグを修正しました。
5. MJXで ``efc_address`` が ``get_data`` で不正に計算されることがあったバグを修正しました。


Version 3.1.0 (December 12, 2023)
---------------------------------

全般
^^^^^^^
1. 符号付き距離関数（SDF）衝突の収束を改善しました。直線探索と新しい目的関数を使用して最適化を行います。これにより、接触の
   発見に必要な初期点の数を減らすことができ、非常に小さいまたは大きいジオムサイズに対してよりロバストになりました。
2. MJCFに :ref:`frame<frame>` を追加しました。 :ref:`body<body>` を必要とせずに、直接の子要素に対して純粋な座標変換を
   定義する :ref:`メタ要素<meta-element>` です。
3. :ref:`position<actuator-position>` および :ref:`intvelocity<actuator-intvelocity>` アクチュエータに :at:`kv` 属性を
   追加しました。アクチュエータが適用する減衰を指定するためのものです。これを使用して、参照速度0のPDコントローラを実装できます。
   この属性を使用する場合は、implicitfastまたはimplicitの :ref:`積分器<geIntegration>` を使用することを推奨します。

プラグイン
^^^^^^^^^^^
4. アクチュエータプラグインが ``mjData.plugin_state`` ではなく ``mjData.act`` の活性化変数を内部状態として使用できるように
   しました。アクチュエータプラグインは活性化変数を計算する :ref:`コールバック<mjpPlugin>` を指定でき、組み込みの
   :ref:`dyntype<actuator-plugin-dyntype>` アクチュエータダイナミクスと共に使用できるようになりました。
5. `pid <https://github.com/deepmind/mujoco/blob/main/plugin/actuator/README.md>`__ アクチュエータプラグインを追加しました。
   ネイティブのMuJoCoアクチュエータでは利用できない積分項を実装する、設定可能なPIDコントローラです。

MJX
^^^
6.  MJXに ``site_xpos`` と ``site_xmat`` を追加しました。
7. ``device_put`` と ``device_get_into`` を置き換える ``put_data`` 、 ``put_model`` 、 ``get_data`` を追加しました。
   ``device_put`` と ``device_get_into`` は非推奨となる予定です。これらの新しい関数は ``efc_J`` などの中間計算結果である
   フィールドを正しく変換します。

バグ修正
^^^^^^^^^
8. 四足歩行ロボットでボディ中心のデカルトアクチュエータを使用する場合のように、可動refsiteを使用したデカルトアクチュエーションの
   バグを修正しました。この修正前は、そのようなアクチュエータは運動量の非保存を引き起こす可能性がありました。
9. :ref:`simulate<saSimulate>` でフレックスを使用できなかったバグを修正しました。
10. ピン固定されたフレックス頂点との組み合わせで弾性プラグインを使用できなかったバグを修正しました。
11. ``SYSTEM_VERSION_COMPAT`` が設定されているx86_64システムをサポートするため、macOS 10.16をターゲットとしたPythonホイールを
    リリースしました。最低サポートバージョンは引き続き11.0ですが、これらのユーザーの互換性を修正するためにこれらのホイールを
    リリースしています。 :issue:`1213` を参照。
12. メッシュの質量計算を修正しました: 慣性ボックスを使用した近似の代わりに、正しいメッシュ体積を使用するようにしました。

Version 3.0.1 (November 15, 2023)
---------------------------------

全般
^^^^^^^
1. ``mjData.qfrc_passive`` の全受動力のサブ項を :ref:`mjData` に追加しました:
   ``qfrc_{spring, damper, gravcomp, fluid}`` 。これらのベクトルの合計は ``qfrc_passive`` と等しくなります。

.. youtube:: H9qG9Zf2W44
   :aspect: 16:7
   :align: right
   :width: 240px

2. :ref:`actuatorgroupdisable<option-actuatorgroupdisable>` 属性と関連する
   :ref:`mjOption.disableactuator<mjOption>` 整数ビットフィールドを追加しました。
   :ref:`group<actuator-general-group>` に応じてアクチュエータのセットを実行時に無効化するために使用できます。
   :issue:`1092` を修正。 :ref:`CActDisable` を参照。

   - 最初の6つのアクチュエータグループは :ref:`simulate<saSimulate>` ビューアでトグル可能です。
     `サンプルモデル
     <https://github.com/google-deepmind/mujoco/blob/main/test/engine/testdata/actuation/actuator_group_disable.xml>`__
     と右の関連するスクリーンキャプチャを参照してください。

3. ``mjMAXUIITEM`` （Simulateのセクションごとの最大UI要素数）を200に増加しました。

MJX
^^^
4. Newtonソルバー（ :ref:`mjtSolver` の ``mjSOL_NEWTON`` ）のサポートを追加しました。NewtonソルバーはGPU上の
   シミュレーションを大幅に高速化します:

   .. list-table:: ステップ/秒、共役勾配法 vs. Newton（A100）
      :header-rows: 1
      :align: left

      * - モデル
        - CG
        - Newton
        - 高速化
      * - `Humanoid <https://github.com/google-deepmind/mujoco/tree/56006355b29424658b56aedb48a4269bd4361c68/mjx/mujoco/mjx/benchmark/model/humanoid>`__
        - 640,000
        - 1,020,000
        - **1.6 x**
      * - `Barkour v0 <https://github.com/google-deepmind/mujoco/tree/56006355b29424658b56aedb48a4269bd4361c68/mjx/mujoco/mjx/benchmark/model/barkour_v0>`__
        - 1,290,000
        - 1,750,000
        - **1.35 x**
      * - `Shadow Hand <https://github.com/google-deepmind/mujoco/tree/56006355b29424658b56aedb48a4269bd4361c68/mjx/mujoco/mjx/benchmark/model/shadow_hand>`__
        - 215,000
        - 270,000
        - **1.25 x**

   Humanoidは標準のMuJoCoヒューマノイドで、
   `Google Barkour <https://blog.research.google/2023/05/barkour-benchmarking-animal-level.html>`__ とShadow Handは
   どちらも :ref:`MuJoCo Menagerie<Menagerie>` で利用可能です。
5. ジョイント等式制約（ :ref:`mjtEq` の ``mjEQ_JOINT`` ）のサポートを追加しました。
6. ``jnt_limited`` が混在するジョイントが正しく拘束されていなかったバグを修正しました。
7. ``device_put`` の型バリデーションをより詳細にしました（ :issue:`1113` を修正）。
8. リミットのないジョイントの空のEFC行を ``MJX`` から削除しました（ :issue:`1117` を修正）。
9. 一部のキネマティックツリーレイアウトで不正なスムースダイナミクスを引き起こしていた ``scan.body_tree`` のバグを修正しました。

Pythonバインディング
^^^^^^^^^^^^^^^^^^^^^^^

10. macOSの ``mjpython`` ランチャーがApple Command Line ToolsのPythonインタプリタで動作するように修正しました。
11. プラグインを使用するモデルの ``mujoco.MjData`` インスタンスをコピーする際のクラッシュを修正しました。その ``MjData``
    インスタンスの作成に使用されたモデルへの参照である ``model`` 属性を ``MjData`` に導入しました。

Simulate
^^^^^^^^
12. :ref:`simulate<saSimulate>` : 「Pause update」、「Fullscreen」、「VSync」ボタンの正しいハンドリング。

ドキュメント
^^^^^^^^^^^^^

.. youtube:: cE3s_IfO4g4
   :aspect: 16:7
   :align: right
   :width: 240px

13. プロシージャルカメラ制御の例を提供するセルを `チュートリアルcolab <https://github.com/google-deepmind/mujoco#getting-started>`__ に追加しました。
14. :ref:`UI` フレームワークのドキュメントを追加しました。
15. ドキュメントの誤字およびサポートされているフィールドを修正しました（ :issue:`1105` および :issue:`1106` を修正）。


バグ修正
^^^^^^^^^
16. :ref:`torquescale<equality-weld-torquescale>` で変更された溶接制約に関連するバグを修正しました。


Version 3.0.0 (October 18, 2023)
---------------------------------

新機能
^^^^^^^^^^^^

1. 新しい :doc:`mjx` （MJX）Pythonモジュールによる、GPUおよびTPU上でのシミュレーションを追加しました。Pythonユーザーは、Google TPUまたは自身のアクセラレータハードウェア上で、毎秒数百万ステップのMuJoCoシミュレーションをネイティブに実行できるようになりました。

   - MJXはオンデバイス強化学習アルゴリズムと連携するよう設計されています。このColabノートブックでは、MJXと強化学習を使用してヒューマノイドロボットと四足歩行ロボットの歩行を訓練する方法を紹介しています: |colab|
   - MJX APIはMuJoCoと互換性がありますが、このリリースではいくつかの機能が欠けています。詳細は :ref:`MJX機能パリティ <MjxFeatureParity>` の概要を参照してください。

.. |colab| image:: https://colab.research.google.com/assets/colab-badge.png
           :target: https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/mjx/tutorial.ipynb

.. youtube:: QewlEqIZi1o
   :aspect: 16:7
   :align: right
   :width: 240px

2. 新しい符号付き距離場（SDF）衝突プリミティブを追加しました。SDFは任意の形状を取ることができ、凸形状に制限されません。衝突点は、2つの衝突するSDFの最大値を勾配降下法で最小化することにより検出されます。

   - 陰的ジオメトリを定義するための新しいSDFプラグインを追加しました。プラグインは、クエリポイントにおけるSDFとその勾配を計算するメソッドを定義する必要があります。詳細は :ref:`ドキュメント<exWriting>` を参照してください。

.. youtube:: ra2bTiZHGlw
   :aspect: 16:7
   :align: right
   :width: 240px

3. 変形可能オブジェクトの定義に使用される ``flex`` と呼ばれる新しい低レベルモデル要素を追加しました。これらの `単体複体 <https://en.wikipedia.org/wiki/Simplicial_complex>`__ は次元1、2、または3を取ることができ、それぞれ伸縮可能な線分、三角形、四面体に対応します。フレックスの定義には2つの新しいMJCF要素が使用されます。トップレベルの :ref:`deformable<deformable>` セクションには低レベルのフレックス定義が含まれています。 :ref:`flexcomp<body-flexcomp>` 要素は :ref:`composite<body-composite>` と同様の便利なマクロで、変形可能オブジェクトの作成をサポートし、GMSH四面体ファイル形式に対応しています。

   - `shell <https://github.com/deepmind/mujoco/blob/main/plugin/elasticity/shell.cc>`__ パッシブフォースプラグインを追加しました。事前計算された定数ヘッセ行列（余接オペレータ）を使用して曲げ力を計算します。

   **注意**: この機能はまだ開発中であり、変更される可能性があります。特に、変形可能オブジェクトの機能は現在、 :ref:`deformable<CDeformable>` と :ref:`composite<CComposite>` の両方を通じて利用可能であり、どちらもファーストパーティの `弾性プラグイン <https://github.com/google-deepmind/mujoco/tree/main/plugin/elasticity>`__ によって変更可能です。今後、これらの機能の一部が統合される予定です。

.. youtube:: Vc1tq0fFvQA
   :aspect: 16:7
   :align: right
   :width: 240px

4. :ref:`mj_island` による制約アイランド探索を追加しました。制約アイランドは、相互作用しない制約と自由度の互いに素な集合です。現在アイランドをサポートしている唯一のソルバーは :ref:`CG<option-solver>` です。アイランド探索は新しい :ref:`有効化フラグ<option-flag-island>` を使用して有効にできます。アイランド探索が有効な場合、ジオム、接触、テンドンは対応するアイランドに従って色分けされます（動画を参照）。アイランド探索は現在、変形可能オブジェクトを含むモデルでは無効化されています（前項を参照）。

5. :ref:`mjThreadPool` と :ref:`mjTask` を追加しました。これにより、MuJoCoエンジンパイプライン内でのマルチスレッド操作が可能になります。エンジン内部スレッディングが有効な場合、以下の操作がマルチスレッド化されます:

   - アイランド制約解決（アイランド探索が :ref:`有効<option-flag-island>` で、 :ref:`CGソルバー<option-solver>` が選択されている場合）。 `22 humanoids <https://github.com/deepmind/mujoco/blob/main/model/humanoid/22_humanoids.xml>`__ モデルでは、シングルスレッドシミュレーションと比較して3倍の高速化が確認されています。
   - 慣性関連の計算と衝突検出が並列に実行されます。

   エンジン内部スレッディングは開発中であり、現在はファーストパーティコードで :ref:`testspeed<saTestspeed>` ユーティリティを通じてのみ利用可能で、 ``npoolthread`` フラグで公開されています。

6. OBJファイルから :ref:`composite<body-composite>` パーティクルを初期化する機能を追加しました。 :issue:`642` および :issue:`674` を修正。

全般
^^^^^^^

.. admonition:: Breaking API changes
   :class: attention

   7. マクロ ``mjMARKSTACK`` と ``mjFREESTACK`` を削除しました。

      **移行方法:** これらのマクロは新しい関数 :ref:`mj_markStack` と :ref:`mj_freeStack` に置き換えられました。これらの関数は :ref:`mjData スタック<siStack>` を完全にカプセル化された方法で管理します（つまり、呼び出し元にローカル変数を導入しません）。

   8. ``mj_stackAlloc`` を :ref:`mj_stackAllocNum` に名前変更しました。新しい関数 :ref:`mj_stackAllocByte` は任意のバイト数を確保し、返されるポインタのアライメントを指定するための追加引数があります。

      **移行方法:** ``mjtNum`` 配列を確保する機能は :ref:`mj_stackAllocNum` で利用可能になりました。

   9. :ref:`mjModel` と :ref:`mjData` の ``nstack`` フィールドを ``narena`` に名前変更しました。 ``narena`` 、 ``pstack`` 、 ``maxuse_stack`` を :ref:`mjtNum` |-| の数ではなくバイト数をカウントするように変更しました。

   10. ソルバー診断情報を収集するために使用される配列 :ref:`mjData.solver<mjData>` を変更しました。この :ref:`mjSolverStat` 構造体の配列は、長さ ``mjNISLAND * mjNSOLVER`` の行列として解釈されるようになりました。長さ ``mjNSOLVER`` の各行には、各制約アイランドの個別のソルバー統計情報が含まれます。ソルバーがアイランドを使用しない場合、行0のみが埋められます。

       - 新しい定数 :ref:`mjNISLAND<glNumeric>` は20に設定されました。
       - :ref:`mjNSOLVER<glNumeric>` は1000から200に削減されました。
       - :ref:`mjData.solver_nisland<mjData>` を追加: ソルバーが実行されたアイランドの数。
       - ``mjData.solver_iter`` を ``solver_niter`` に名前変更しました。このメンバーと ``mjData.solver_nnz`` は、長さ ``mjNISLAND`` の整数ベクトルになりました。

   11. ``mjOption.collision`` および関連する ``option/collision`` 属性を削除しました。

       **移行方法:**

       - ``<option collision="all"/>`` を持つモデルの場合、その属性を削除してください。
       - ``<option collision="dynamic"/>`` を持つモデルの場合、すべての :ref:`pair<contact-pair>` 要素を削除してください。
       - ``<option collision="predefined"/>`` を持つモデルの場合、まずモデル内のすべての :ref:`contype<body-geom-contype>` と :ref:`conaffinity<body-geom-conaffinity>` 属性を削除し、次にそれらを |br| ``<default> <geom contype="0" conaffinity="0"/> </default>`` を使用してグローバルに ``0`` に設定することで、すべての動的衝突（contype/conaffinityで決定）を無効にしてください。

   12. :at:`rope` と :at:`cloth` のコンポジットオブジェクトを削除しました。

       **移行方法:** ユーザーは :at:`cable` と :at:`shell` の弾性プラグインを使用してください。

   13. ユーザー入力変数 :ref:`mjData.eq_active<mjData>` を追加しました。等式制約の有効/無効を切り替えるためのものです。 ``mjModel.eq_active`` を :ref:`mjModel.eq_active0<mjModel>` に名前変更しました。これは「 ``mjData.eq_active`` の初期値」というセマンティクスを持つようになりました。 :issue:`876` を修正。

       **移行方法:** ``mjModel.eq_active`` の使用を ``mjData.eq_active`` に置き換えてください。

   14. :ref:`autolimits<compiler-autolimits>` のデフォルト値を "false" から "true" に変更しました。これは軽微な破壊的変更です。影響を受ける可能性があるのは、 "range" が定義されているが "limited" が設定されていない要素を持つモデルです。このようなモデルはバージョン2.2.2（2022年7月）以降読み込むことができません。

15. 新しい :ref:`dyntype<actuator-general-dyntype>` として ``filterexact`` を追加しました。オイラー積分ではなく厳密な公式を使用して一次フィルタの状態を更新します。
16. アクチュエータ属性 :ref:`actearly<actuator-general-actearly>` を追加しました。アクチュエータ力の半陰的積分を使用します: 現在のアクチュエータ力の計算に次のステップのアクチュエータ状態を使用します。
17. 前バージョンで導入された ``actuatorforcerange`` と ``actuatorforcelimited`` を、それぞれ :ref:`actuatorfrcrange<body-joint-actuatorfrcrange>` と :ref:`actuatorfrclimited<body-joint-actuatorfrclimited>` に名前変更しました。
18. フラグ :ref:`eulerdamp<option-flag-eulerdamp>` を追加しました。オイラー積分器でのジョイント減衰の陰的積分を無効にします。詳細は :ref:`数値積分<geIntegration>` セクションを参照してください。
19. フラグ :ref:`invdiscrete<option-flag-invdiscrete>` を追加しました。 ``RK4`` 以外のすべての :ref:`積分器<option-integrator>` に対して離散時間逆動力学を有効にします。詳細はフラグのドキュメントを参照してください。
20. CGソルバーとニュートンソルバーの直線探索停止条件を調整するための :ref:`ls_iterations<option-ls_iterations>` と :ref:`ls_tolerance<option-ls_tolerance>` オプションを追加しました。パフォーマンスチューニングに有用です。
21. メッシュアセットに適用される正規化変換を格納するため、 :ref:`mjModel` に ``mesh_pos`` と ``mesh_quat`` フィールドを追加しました。 :issue:`409` を修正。
22. カメラの :ref:`resolution<body-camera-resolution>` 属性と :ref:`camprojection<sensor-camprojection>` センサーを追加しました。カメラの解像度が正の値に設定されている場合、カメラ投影センサーはターゲットサイトの位置をカメラ画像上に投影し、ピクセル座標で報告します。
23. :ref:`camera<body-camera>` キャリブレーション属性を追加しました:

    - 新しい属性は :ref:`resolution<body-camera-resolution>` 、 :ref:`focal<body-camera-focal>` 、 :ref:`focalpixel<body-camera-focalpixel>` 、 :ref:`principal<body-camera-principal>` 、 :ref:`principalpixel<body-camera-principalpixel>` 、 :ref:`sensorsize<body-camera-sensorsize>` です。
    - これらの属性が指定されている場合、 :ref:`mjVIS_CAMERA<mjtVisFlag>` 可視化フラグを使用してキャリブレーションされた錐台を可視化できます。以下の `サンプルモデル <https://github.com/deepmind/mujoco/blob/main/test/engine/testdata/vis_visualize/frustum.xml>`__ を参照してください。
    - これらの属性はオフラインレンダリングにのみ適用され、インタラクティブな可視化には影響しないことに注意してください。
24. より良い深度精度のために、反転Zレンダリングを実装しました。列挙型 :ref:`mjtDepthMap` が追加され、値 ``mjDEPTH_ZERONEAR`` と ``mjDEPTH_ZEROFAR`` があります。これらは :ref:`mjrContext` の新しい ``readDepthMap`` 属性を設定するために使用でき、 :ref:`mjr_readPixels` が返す深度が ``znear`` から ``zfar`` にどのようにマッピングされるかを制御します。コントリビューション :pr:`978` （ `Levi Burner <https://github.com/aftersomemath>`__ ）。
25. コードサンプル ``testxml`` を削除しました。このユーティリティが提供していた機能は `WriteReadCompare <https://github.com/google-deepmind/mujoco/blob/main/test/xml/xml_native_writer_test.cc>`__ テストに実装されています。
26. コードサンプル ``derivative`` を削除しました。この機能は :ref:`mjd_transitionFD` により提供されています。

Pythonバインディング
^^^^^^^^^^^^^^^^^^^^^^^

27. ``update_scene`` を無効なカメラ名で呼び出した場合にデフォルトカメラが使用されていた :issue:`870` を修正しました。
28. :ref:`パッシブビューア<PyViewerPassive>` ハンドルに ``user_scn`` を追加しました。ユーザーがカスタム可視化ジオムを追加できるようになります（ :issue:`1023` ）。
29. 関数 ``viewer.launch`` と ``viewer.launch_passive`` にオプションのブーリアンキーワード引数 ``show_left_ui`` と ``show_right_ui`` を追加しました。UIパネルを非表示にした状態でビューアを起動できます。

Simulate
^^^^^^^^

.. youtube:: YSvWn_poqWs
   :aspect: 16:7
   :align: right
   :width: 240px

30. :ref:`simulate<saSimulate>` およびマネージド :ref:`Pythonビューア<PyViewerManaged>` に **状態履歴** メカニズムを追加しました。状態履歴はHistoryスライダーをスクラブすることで、またより精密には左右の矢印キーで閲覧できます。スクリーンキャプチャを参照:

31. ``LOADING...`` ラベルが正しく表示されるようになりました。コントリビューション :pr:`1070` （ `Levi Burner <https://github.com/aftersomemath>`__ ）。

ドキュメント
^^^^^^^^^^^^^

.. youtube:: nljr0X79vI0
   :aspect: 16:7
   :align: right
   :width: 240px

32. 流体力モデリングの :doc:`詳細なドキュメント <computation/fluid>` と、楕円体ベースの流体モデルを使用した `回転するカード <https://github.com/google-deepmind/mujoco/blob/main/model/cards/cards.xml>`__ の説明的なサンプルモデルを追加しました。

バグ修正
^^^^^^^^^

33. :ref:`ジオムのmargin<body-geom-margin>` がミッドフェーズ衝突ツリーの構築時に無視されていたバグを修正しました。

34. 溶接等式制約に対して ``efc_diagApprox`` で不正な値が生成されていたバグを修正しました。


Version 2.3.7 (July 20, 2023)
-----------------------------

全般
^^^^^^^

1. 球-円筒の接触用プリミティブコライダーを追加しました。以前このペアは汎用の凸-凸コライダーを使用していました。
#. ジョイントにおけるアクチュエータ力の合計をクランプするための :ref:`joint-actuatorforcerange<body-joint-actuatorfrcrange>` と、ジョイントに適用される合計アクチュエーション力を測定するための :ref:`sensor-jointactuatorfrc<sensor-jointactuatorfrc>` を追加しました。ジョイントレベルのアクチュエータ力クランプの最も重要なユースケースは、 :ref:`デカルトアクチュエータ<actuator-general-refsite>` の力がジョイントの個々のモーターで実現可能であることを保証することです。詳細は :ref:`CForceRange` を参照してください。
#. hfield、テクスチャ、メッシュアセットにオプションの ``content_type`` 属性を追加しました。この属性は、ファイル拡張子からタイプを取得する代わりに、アセットファイルのタイプを決定するために使用されるフォーマットされた `メディアタイプ <https://www.iana.org/assignments/media-types/media-types.xhtml>`_ （以前はMIMEタイプとして知られていた）文字列をサポートします。
#. クォータニオンの :ref:`減算<mjd_subQuat>` と :ref:`積分<mjd_quatIntegrate>` （角速度による回転）の解析的導関数を追加しました。導関数は3D接空間内にあります。
#. ``mjv_makeConnector`` と同一の機能を持つ :ref:`mjv_connector` を追加しました。ただし、より便利な "from-to" 引数パラメータ化になっています。 ``mjv_makeConnector`` は非推奨になりました。
#. サポートされる最も古いMacOSをバージョン10.12から11に引き上げました。MacOS 11はAppleがまだメンテナンスしている最も古いバージョンです。

Pythonバインディング
^^^^^^^^^^^^^^^^^^^^^^^

7. :ref:`パッシブビューア<PyViewerPassive>` ハンドルが ``update_hfield`` 、 ``update_mesh`` 、 ``update_texture`` メソッドを公開するようになり、ユーザーがレンダリング可能なアセットを更新できるようになりました。（Issues :issue:`812` 、 :issue:`958` 、 :issue:`965` ）。
#. :ref:`パッシブビューア<PyViewerPassive>` でカスタムキーボードイベントコールバックを指定できるようになりました（ :issue:`766` ）。
#. Pythonの終了時にパッシブビューアが実行中の場合のGLFWクラッシュを修正しました（ :issue:`790` ）。

モデル
^^^^^^

10. シンプルな `car <https://github.com/google-deepmind/mujoco/blob/main/model/car/car.xml>`__ サンプルモデルを追加しました。


Version 2.3.6 (June 20, 2023)
-----------------------------

.. note::
   MuJoCo 2.3.6はPython 3.7を公式にサポートする最後のバージョンです。

.. youtube:: ZppeDArq6AU
   :align: right
   :width: 240px

モデル
^^^^^^

1. `3x3x3 cube <https://github.com/google-deepmind/mujoco/blob/main/model/cube/cube_3x3x3.xml>`__ サンプルモデルを追加しました。
   詳細は `README <https://github.com/google-deepmind/mujoco/blob/main/model/cube/README.md>`__ を参照してください。

バグ修正
^^^^^^^^^

2. メッシュのボリュームが無効な場合に、メッシュのバウンディングボックスと座標フレームの計算が不正になるバグを修正しました。そのような場合、MuJoCoは :ref:`shellinertia<body-geom-shellinertia>` が ``true`` に等しい場合のみ非水密ジオメトリを受け入れるようになりました。
#. テンドンの減衰と流体力の導関数を計算するために使用されるスパースヤコビアン乗算ロジックを修正しました。これは :ref:`implicitおよびimplicitfast積分器<geIntegration>` の動作に影響します。
#. ジオムの可視化規則に合わせて :ref:`mj_ray` を修正しました：

   - 平面とハイトフィールドが ``geom_group`` および ``flg_static`` 引数を尊重するようになりました。この変更前は、レイは平面とハイトフィールドに無条件で交差していました。
   - ``flg_static`` がワールドボディの直接の子であるものだけでなく、すべての静的ジオムに適用されるようになりました。

.. youtube:: hqIMTNGaLF4
   :align: right
   :width: 240px

プラグイン
^^^^^^^^^^^^

5. タッチグリッドセンサープラグインを追加しました。詳細は `ドキュメント <https://github.com/google-deepmind/mujoco/blob/main/plugin/sensor/README.md>`__ を参照してください。関連する `touch_grid.xml <https://github.com/google-deepmind/mujoco/blob/main/model/plugin/sensor/touch_grid.xml>`__ サンプルモデルもあります。このプラグインには `シーン内可視化 <https://youtu.be/0LOJ3WMnqeA>`__ が含まれています。

Simulate
^^^^^^^^

.. youtube:: mXVPbppGk5I
   :aspect: 16:7
   :align: right
   :width: 240px

6. SimulateのUIにVisualizationタブを追加しました。これは :ref:`visual<visual>` MJCF要素の要素に対応しています。GUIで値を変更した後、保存されたXMLには新しい値が含まれます。 :ref:`mjStatistic` の変更可能なメンバー（ :ref:`extent<statistic-extent>` 、 :ref:`meansize<statistic-meansize>` 、 :ref:`center<statistic-center>` ）はコンパイラによって計算されるため、デフォルト値を持ちません。これらの属性が保存されたXMLに表示されるようにするには、読み込まれたXMLで値を指定する必要があります。

.. image:: images/changelog/simulate_text_width.png
   :align: right
   :width: 380px
   :alt: Before / After

7. デフォルトのスペーシングでUI要素のテキスト幅を拡大しました。[before / after]：

全般
^^^^^^^

8. シミュレーション状態を浮動小数点数の連結ベクトルとして取得・設定するための :ref:`mj_getState` と :ref:`mj_setState` を追加しました。詳細は :ref:`State<geState>` セクションを参照してください。
#. :ref:`mjContact.solreffriction<mjContact>` を追加しました。これにより、 :ref:`楕円摩擦円錐<option-cone>` を使用する際に、接触の法線軸と摩擦軸で異なる :ref:`solref<CSolver>` パラメータを使用できるようになります。この属性は弾性摩擦衝突に必要です。 `弾性ゴムボール <https://www.youtube.com/watch?v=uFLJcRegIVQ&t=3s>`__ のスピンバウンスリコイル動作を模倣する関連 `サンプルモデル <https://github.com/google-deepmind/mujoco/blob/main/test/engine/testdata/spin_recoil.xml>`__ を参照してください。これは現在、 :ref:`solreffriction<contact-pair-solreffriction>` 属性を使用して明示的な :ref:`接触ペア<contact-pair>` でのみサポートされている高度なオプションです。
#. 有限差分による逆動力学の導関数のための :ref:`mjd_inverseFD` を追加しました。
#. バンド＋密行列（"arrowhead"行列）に対する演算関数を追加しました。このような行列は直接軌道最適化を行う際に一般的です。詳細は :ref:`mju_cholFactorBand` のドキュメントを参照してください。
#. 単一の点から放射される複数のレイを交差させるための :ref:`mj_multiRay` 関数を追加しました。これは :ref:`mj_ray` を複数回呼び出すよりも大幅に高速です。
#. メッシュ面のバウンディングボリューム階層を使用して、レイ-メッシュ衝突が最大10倍高速になりました。
#. ``mjMAXUIITEM`` （SimulateのセクションあたりのUI要素の最大数）を100に増加しました。
#. リソースプロバイダーの :ref:`ドキュメント<exProvider>` を追加しました。
#. 有限サポートシグモイド :math:`s \colon \mathbf R \rightarrow [0, 1]` である :ref:`mju_sigmoid` の式を変更しました。以前は、滑らかな部分は2つの接続された2次関数で構成され、1回連続微分可能でした。現在は単一の5次関数で、2回連続微分可能です：

   .. math::
      s(x) =
      \begin{cases}
         0,                    &       & x \le 0  \\
         6x^5 - 15x^4 + 10x^3, & 0 \lt & x \lt 1  \\
         1,                    & 1 \le & x \qquad
      \end{cases}

17. 筋肉アクチュエータにオプションの :ref:`tausmooth<actuator-muscle-tausmooth>` 属性を追加しました。正の値の場合、筋肉の活性化/非活性化の時定数 :math:`\tau` は :ref:`mju_sigmoid` を使用して、 `Millard et al. (2013) <https://doi.org/10.1115/1.4023390>`__ 筋肉モデルによって与えられる2つの極値間をtausmoothの幅の範囲内で滑らかに遷移します。詳細は :ref:`筋肉アクチュエータ<CMuscle>` を参照してください。関連して、 :ref:`mju_muscleDynamics` は新しい平滑化幅パラメータを追加し、2つではなく3つのパラメータを取るようになりました。
#.  パブリックCマクロ定義をmujoco.hから `mjmacro.h <https://github.com/google-deepmind/mujoco/blob/main/include/mujoco/mjmacro.h>`__ という新しいパブリックヘッダファイルに移動しました。新しいファイルはmujoco.hからインクルードされるため、この変更は既存のユーザーコードを壊しません。
#.  ``mjData`` のスタックとアリーナからの割り当て時にメモリバグを検出するための `Address Sanitizer (ASAN) <https://clang.llvm.org/docs/AddressSanitizer.html>`__ と `Memory Sanitizer (MSAN) <https://clang.llvm.org/docs/MemorySanitizer.html>`__ の計装を追加しました。
#.  ``pstack`` と ``parena`` を ``mj_printData`` の出力から削除しました。これらは計装されたビルドの診断パディングの影響を受ける ``mjData`` アロケータの実装詳細であるためです。
#.  ``mj_activate`` と ``mj_deactivate`` 関数を削除しました。これらはMuJoCoがクローズドソースだった時代の古いユーザーコードとの互換性のために残されていましたが、オープンソース化以降はno-op関数でした。


Version 2.3.5 (April 25, 2023)
------------------------------

バグ修正
^^^^^^^^^

1. :ref:`mjVFS` が使用されている場合にOBJファイルとPNGファイルがディスクから読み込めなくなるアセット読み込みバグを修正しました。
#. macOSでマウス摂動が適用された際にPythonパッシブビューアで時折発生するセグメンテーションフォルトを修正しました。

プラグイン
^^^^^^^^^^^^

3. :ref:`mjpPlugin` の ``visualize`` コールバックが入力引数として :ref:`mjvOption` を受け取るようになりました。


Version 2.3.4 (April 20, 2023)
------------------------------

.. note::

   このバージョンには、 ``mjVFS`` が使用されている場合にOBJファイルとPNGファイルがディスクから読み込めなくなるアセット読み込みバグがあります。ユーザーは代わりにバージョン2.3.5にスキップすることをお勧めします。

全般
^^^^^^^

1. :ref:`compiler/coordinate<compiler-coordinate>` 属性の "global" 設定を削除しました。このめったに使われない設定はコンパイラロジックを複雑にし、将来の改善を妨げていました。このオプションを使用していた古いモデルを変換するには、MuJoCo 2.3.3以前で読み込んで保存してください。

.. image:: images/changelog/ellipsoidinertia.gif
   :align: right
   :width: 240px

2. 等価ボディ慣性をデフォルトのボックスの代わりに楕円体で可視化するための :ref:`visual-global<visual-global>` フラグ :ref:`ellipsoidinertia<visual-global-ellipsoidinertia>` を追加しました。
#. ミッドフェーズとブロードフェーズの衝突統計を :ref:`mjData` に追加しました。
#. :ref:`エンジンプラグイン<exPlugin>` のドキュメントを追加しました。
#. ``introspect`` モジュールに構造体情報を追加しました。
#. :ref:`リソースプロバイダー<exProvider>` と呼ばれる新しい拡張メカニズムを追加しました。この拡張可能なメカニズムにより、MuJoCoはローカルOSファイルシステムや :ref:`仮想ファイルシステム<Virtualfilesystem>` 以外のデータソースからアセットを読み込むことができます。

Pythonバインディング
^^^^^^^^^^^^^^^^^^^^^^^

7. macOSでのオフスクリーンレンダリングがメインスレッドに制限されなくなりました。これは、CocoaのNSOpenGLに依存するGLFWを経由する代わりに、低レベルのCore OpenGL (CGL) APIを使用してOpenGLコンテキストを作成することで実現されています。結果として得られるコンテキストはCocoaウィンドウに紐付けられておらず、そのためメインスレッドにも紐付けられていません。
#. ``viewer.launch_passive`` と ``viewer.launch_repl`` の競合状態を修正しました。これらの関数は以前、内部の ``mj_forward`` 呼び出しが完了する前にリターンすることがありました。これにより、ユーザーコードが内部の ``mj_forward`` と並行して物理状態を変更でき、例えば `MuJoCoスタックオーバーフローエラー <https://github.com/google-deepmind/mujoco/issues/783>`__ や `セグメンテーションフォルト <https://github.com/google-deepmind/mujoco/issues/790>`__ が発生する可能性がありました。
#. ``viewer.launch_passive`` 関数がビューアとのインタラクションに使用できるハンドルを返すようになりました。パッシブビューアは、物理状態の更新を取得するためにハンドルに対する明示的な ``sync`` 呼び出しも必要とするようになりました。これは視覚的なアーティファクトを引き起こす可能性のある競合状態を回避するためです。詳細は :ref:`ドキュメント<PyViewerPassive>` を参照してください。
#. ``viewer.launch_repl`` 関数は ``launch_passive`` によって機能が置き換えられたため削除されました。
#. 新しい ``introspect`` メタデータを通じて発見された少数の欠落していた構造体フィールドを追加しました。

バグ修正
^^^^^^^^^

12. 新しいimplicitfast積分器における楕円体ベースの流体モデル力の処理のバグを修正しました。
#.  `mj_copyData` における不要なアリーナ全体のコピーを削除しました。これによりコピー操作が大幅に `遅くなる <https://github.com/google-deepmind/mujoco/issues/568>`__ 可能性がありました。
#. :ref:`shellinertia<body-geom-shellinertia>` がレガシーなボリューム計算にのみ使用される ``exactmeshinertia`` を無視するようにしました（ `#759 <https://github.com/google-deepmind/mujoco/issues/759>`__ ）。


Version 2.3.3 (March 20, 2023)
------------------------------

全般
^^^^^^^

1. 陰的積分の改善：

   - RNEアルゴリズムの導関数がスパース数学を使用して計算されるようになり、 :ref:`implicit積分器<geIntegration>` を使用する際の大規模モデルで大幅な速度改善が得られるようになりました。
   - ``implicitfast`` と呼ばれる新しい積分器が追加されました。これは既存のimplicit積分器に似ていますが、コリオリ力と遠心力の導関数をスキップします。詳細な動機と議論については :ref:`数値積分<geIntegration>` セクションを参照してください。implicitfast積分器はすべての新しいモデルに推奨され、将来のバージョンでデフォルトの積分器になる予定です。

   以下の表は、異なる積分器を使用した627自由度の `humanoid100 <https://github.com/google-deepmind/mujoco/blob/main/model/humanoid/humanoid100.xml>`__ モデルの計算コストを示しています。"implicit (old)" は密なRNE導関数を使用し、"implicit (new)" は上記のスパース化後のものです。タイミングはAMD 3995WX CPUの単一コアで測定されました。

.. csv-table::
   :header: "タイミング", "Euler", "implicitfast", "implicit (new)", "implicit (old)"
   :widths: auto
   :align: left

   1ステップ (ms),  0.5,   0.53,  0.77,  5.0
   ステップ/秒,   2000,  1900,  1300,  200

.. image:: images/computation/midphase.gif
   :align: right
   :width: 350px

.. _midphase:

2. ボディペア内のジオムを枝刈りするための衝突ミッドフェーズを追加しました。詳細は :ref:`ドキュメント<coSelection>` を参照してください。これはボディの慣性フレームにおける静的AABBバウンディングボリューム階層（BVHバイナリツリー）に基づいています。右のGIFは `こちらのより長い動画 <https://youtu.be/e0babIM8hBo>`__ から切り取られたものです。
#. ``mjd_transitionFD`` 関数は、明示的に要求されない限りセンサー計算をトリガーしなくなりました。
#. :ref:`mjLROpt` 構造体の ``inteval`` 属性のスペルを ``interval`` に修正しました。
#. メッシュのテクスチャマッピングと法線マッピングが頂点あたり1つではなく三角形あたり3つになりました。メッシュの頂点は、この制限を回避するために以前のように複製されなくなりました。
#. スパース制約ヤコビアン行列の非ゼロ要素が事前にカウントされ、行列のメモリ割り当てに使用されるようになりました。例えば、 `humanoid100 <https://github.com/google-deepmind/mujoco/blob/main/model/humanoid/humanoid100.xml>`__ モデルの制約ヤコビアン行列は、以前は約500,000個の ``mjtNum`` を必要としていましたが、現在は約6,000個のみです。非常に大きなモデルもCGソルバーでロードおよび実行できるようになりました。
#. :ref:`mju_error` と :ref:`mju_warning` を可変引数関数（printf風の引数をサポート）に変更しました。 :ref:`mju_error_i` 、 :ref:`mju_error_s` 、 :ref:`mju_warning_i` 、 :ref:`mju_warning_s` 関数は非推奨になりました。
#. 密なメモリ割り当てを必要としないパフォーマンスの高い ``mju_sqrMatTDSparse`` 関数を実装しました。
#. mjDataスタック上でintを割り当てるための正しいサイズを取得する ``mj_stackAllocInt`` を追加しました。スタックメモリ使用量を10%～15%削減します。


Pythonバインディング
^^^^^^^^^^^^^^^^^^^^^^^

10. ``viewer.launch_repl`` 使用時のIPython履歴の破損を修正しました。 ``launch_repl`` 関数はIPythonインタラクティブシェルセッションのシームレスな継続を提供するようになり、実験的機能とは見なされなくなりました。
#.  インタラクティブビューアをパッシブな非ブロッキングモードで起動する ``viewer.launch_passive`` を追加しました。 ``launch_passive`` の呼び出しは即座にリターンし、ユーザーコードの実行を継続できます。ビューアは物理状態への変更を自動的に反映します。（注：この機能は現在実験的/ベータ段階にあり、 :ref:`ビューアのドキュメント<PyViewer>` にはまだ記載されていません。）
#.  macOS用の ``mjpython`` ランチャーを追加しました。これはmacOSで ``viewer.launch_passive`` が機能するために必要です。
#.  ジョイントインデクサーから ``efc_`` フィールドを削除しました。アリーナメモリの導入以降、これらのフィールドはアクティブな制約の数に応じてタイムステップ間で動的にサイズが変化するようになり、ジョイントと ``efc_`` 行の厳密な対応関係が崩れています。
#.  ``mjVisual`` と ``mjvPerturb`` 構造体のバインディングに多数の欠落していたフィールドを追加しました。

Simulate
^^^^^^^^

15. フレームレートが垂直同期トグルが有効な場合に正しくキャップされるように、macOSでの `壊れたVSync <https://github.com/glfw/glfw/issues/2249>`__ の回避策を実装しました。

.. image:: images/changelog/contactlabel.png
   :align: right
   :width: 400px

16. 接触の可視化にオプションのラベルを追加しました。どの2つのジオムが接触しているかを示します（名前が定義されている場合は名前、そうでない場合はid）。これは複雑なシーンで役立ちます。

|br|


Version 2.3.2 (February 7, 2023)
--------------------------------

全般
^^^^^^^

1. 密なメモリ割り当てを必要としない、よりパフォーマンスの高い mju_transposeSparse を実装しました。
   `humanoid100.xml <https://github.com/google-deepmind/mujoco/blob/main/model/humanoid/humanoid100.xml>`__ モデルの制約ヤコビアン行列に対して、
   この関数は35%高速になりました。
#. :ref:`mj_name2id` 関数が、より良いパフォーマンスのために線形探索ではなくハッシュ関数を使用して実装されるようになりました。
#. URDFからジオム名が解析されるようになりました。重複する名前は無視されます。
   ``mj_printData`` の出力に接触しているジオム名が含まれるようになりました。

バグ修正
^^^^^^^^^

4. :at:`shellinertia` が ``true`` の場合に、頂点座標が体積慣性を使用して回転されるのに対してメッシュの向きがシェル慣性の主成分で上書きされるバグを修正しました。シェルの場合でも体積慣性の向きが使用されるようになりました。
#. バウンディングボックスフィッティングオプション :at:`fitaabb` を使用する際のメッシュからプリミティブへのフィッティングにおけるアライメントバグを修正しました。

.. image:: images/changelog/meshfit.png
   :align: right
   :width: 300px

6. Pythonビューアの ``launch_repl`` 機能を修正しました。
#. 時間依存のユーザーコードをサポートするために、 ``mjd_transitionFD`` で ``time`` を正しく設定するようにしました。
#. ``user`` タイプのセンサーが存在する場合のセンサーデータ次元の検証を修正しました。
#. モデルコンパイル中にnullの ``nsensordata`` コールバックが検出された際の不正なプラグインエラーメッセージを修正しました。
#. ``mj_fwdConstraint`` が早期リターンする際にタイマー（ ``TM_END`` ）が正しく終了するようにしました。
#. ``mj_deleteFileVFS`` の無限ループを修正しました。

Simulate
^^^^^^^^

12. simulateのセンサープロットy軸の精度を1桁増加しました
    （ `#719 <https://github.com/google-deepmind/mujoco/issues/719>`_ ）。
#.  ボディラベルが、慣性が可視化されている場合を除き、慣性フレームではなくボディフレームに描画されるようになりました。

プラグイン
^^^^^^^^^^

14. ``reset`` コールバックが、 ``mjData`` 全体ではなく、インスタンス固有の ``plugin_state`` と ``plugin_data`` を引数として受け取るようになりました。 ``reset`` は物理フォワーディング呼び出しが行われる前に ``mj_resetData`` 内部で呼び出されるため、この段階で ``mjData`` から何かを読み取ることはエラーです。
#.  ``mjpPlugin`` の ``capabilities`` フィールドが、これがビットフィールドであることをより明確に示すために ``capabilityflags`` に名前変更されました。


Version 2.3.1 (December 6, 2022)
--------------------------------

Pythonバインディング
^^^^^^^^^^^^^^^^^^^^^^^

1. ``simulate`` GUIが ``mujoco`` Pythonパッケージを通じて ``mujoco.viewer`` として利用可能になりました。
   詳細は :ref:`ドキュメント<PyViewer>` を参照してください。（ `Levi Burner <https://github.com/aftersomemath>`__ による貢献。）
#. MuJoCoチュートリアルColabの ``Renderer`` クラスが、ネイティブPythonバインディングで直接利用可能になりました。

全般
^^^^^^^

3. テンドンの :at:`springlength` 属性が2つの値を取れるようになりました。2つの非減少値が与えられると、 `springlength` はバネ剛性の `デッドバンド <https://en.wikipedia.org/wiki/Deadband>`__ 範囲を指定します。テンドンの長さが2つの値の間にある場合、力は0です。長さがこの範囲外にある場合、力は通常のバネのように動作し、バネの静止長は最も近い :at:`springlength` 値に対応します。これを使用して、制約ではなくバネによって制限が強制されるテンドンを作成できます。これはより安価で解析が容易です。
   `tendon_springlength.xml <https://github.com/google-deepmind/mujoco/blob/main/test/engine/testdata/tendon_springlength.xml>`__
   サンプルモデルを参照してください。

   .. attention::
     これは軽微な破壊的API変更です。 ``mjModel.tendon_lengthspring`` のサイズが ``ntendon x 1`` から ``ntendon x 2`` に変更されました。

   .. youtube:: -PJ6afdETUg
      :align: right
      :height: 150px

#. 状態を持たないアクチュエータが状態を持つアクチュエータの前に来なければならないという要件を削除しました。
#. ユーティリティ関数 :ref:`mju_fill` 、 :ref:`mju_symmetrize` 、 :ref:`mju_eye` を追加しました。
#. 重力補償と浮力を実装する :at:`gravcomp` 属性を :ref:`body<body>` に追加しました。
   `balloons.xml <https://github.com/google-deepmind/mujoco/blob/main/model/balloons/balloons.xml>`__ サンプルモデルを参照してください。
#. ``cable`` プラグインライブラリを ``elasticity`` に名前変更しました。
#. :ref:`汎用アクチュエータ<actuator-general>` に :at:`actdim` 属性を追加しました。1より大きい値は、ネイティブの活性化ダイナミクスがすべてスカラーであるため、dyntype :at-val:`user` でのみ許可されます。2次の活性化ダイナミクスを実装するテスト例を
   `engine_forward_test.cc <https://github.com/google-deepmind/mujoco/blob/main/test/engine/engine_forward_test.cc>`__ に追加しました。
#. particle :ref:`コンポジット<body-composite>` タイプを改善しました。ユーザー指定のジオメトリと複数のジョイントが可能になりました。2つの新しい例を参照してください：
   `particle_free.xml <https://github.com/google-deepmind/mujoco/blob/main/model/composite/particle_free.xml>`__ と
   `particle_free2d.xml <https://github.com/google-deepmind/mujoco/blob/main/model/composite/particle_free2d.xml>`__ 。
#. 非AVX構成のパフォーマンス改善：

   - `restrict <https://en.wikipedia.org/wiki/Restrict>`__ を使用して ``mj_solveLD`` が14%高速化。 `engine_core_smooth_benchmark_test
     <https://github.com/google-deepmind/mujoco/blob/main/test/benchmark/engine_core_smooth_benchmark_test.cc>`__ を参照。
   - 手動ループ展開を使用して ``mju_dotSparse`` が50%高速化。 `engine_util_sparse_benchmark_test
     <https://github.com/google-deepmind/mujoco/blob/main/test/benchmark/engine_util_sparse_benchmark_test.cc>`__ を参照。
#. 新しい :at:`solid` パッシブ力プラグインを追加しました：

   .. youtube:: AGcTGHbbze4
      :align: right
      :height: 150px

   - これは :ref:`コンポジット<body-composite>` パーティクルと互換性のある新しい力場です。
   - 頂点に質量が集中したパーティクルを持つ四面体メッシュを生成します。
   - 有限要素と等価だが座標系に依存しない定式化で表現された区分定数ひずみモデルを使用します。これは、質量・バネモデルのようにエッジの伸長を除くすべての量を事前計算できることを意味します。
   - 小さなひずみ（大きな変位だが小さな変形）にのみ適しています。大きな荷重を受けると四面体が反転する可能性があります。

#. API関数 ``mj_loadPluginLibrary`` と ``mj_loadAllPluginLibraries`` を追加しました。最初の関数はPOSIXシステムでの ``dlopen`` と、Windowsでの ``LoadLibraryA`` と同一です。2番目の関数は指定されたディレクトリ内のすべてのダイナミックライブラリファイルをスキャンし、見つかった各ライブラリをロードします。これらの関数で開かれたダイナミックライブラリは、ロード時に1つ以上のMuJoCoプラグインを登録するものとみなされます。
#. プラグインにオプションの ``visualize`` コールバックを追加しました。これは ``mjv_updateScene`` 中に呼び出されます。このコールバックによりカスタムプラグインの可視化が可能になります。例としてCableプラグインの応力可視化を有効にしました。
#. :ref:`user<sensor-user>` タイプのセンサーが :at:`objtype` 、 :at:`objname` 、 :at:`needstage` を必要としなくなりました。未指定の場合、objtypeは :ref:`mjOBJ_UNKNOWN<mjtObj>` になります。 ``user`` センサーの :at:`datatype` のデフォルトは :at-val:`"real"` に、 :at:`needstage` のデフォルトは :at-val:`"acc"` になりました。
#. URDFインポートでカプセルのサポートを追加しました。
#. macOSで、Apple Siliconマシン上の `Rosetta 2 <https://support.apple.com/en-gb/HT211861>`__ トランスレーション下で実行された場合に、情報量のあるエラーメッセージを発行するようにしました。ビルド済みMuJoCoバイナリはx86-64マシンで `AVX <https://en.wikipedia.org/wiki/Advanced_Vector_Extensions>`__ 命令を使用しますが、これはRosetta 2ではサポートされていません。（このバージョン以前は、ユーザーには謎めいた「Illegal instruction」メッセージしか表示されませんでした。）

バグ修正
^^^^^^^^^

17. ``mj_addFileVFS`` でファイルパスが無視されるバグを修正しました（2.1.4で導入）。

Simulate
^^^^^^^^

18. ``simulate`` アプリケーションがプラグインを検索するディレクトリを ``plugin`` から ``mujoco_plugin`` に名前変更しました。
#.  マウスの力による摂動が、ボディの重心ではなく選択点に適用されるようになりました。


Version 2.3.0 (October 18, 2022)
--------------------------------

全般
^^^^^^^

1. ``mjData`` の ``contact`` 配列と ``efc_`` で始まる配列が ``buffer`` から新しい ``arena`` メモリ空間に移動されました。これらの配列は ``mjData`` の作成時に固定サイズで割り当てられなくなりました。代わりに、 :ref:`mj_forward` の各呼び出し時（具体的には :ref:`mj_collision` と :ref:`mj_makeConstraint` 内で）に正確なメモリ要件が決定され、配列は ``arena`` 空間から割り当てられます。 ``stack`` も利用可能なメモリを ``arena`` と共有するようになりました。この変更により、PGSソルバーを使用しないモデルでの ``mjData`` のメモリフットプリントが削減され、将来的に大幅なメモリ削減が可能になります。
   詳細は :ref:`メモリ割り当て<CSize>` セクションを参照してください。

   .. youtube:: RHnXD6uO3Mg
      :aspect: 16:7
      :align: right
      :height: 150px

#. 線形二次レギュレータを使用してヒューマノイドを片足でバランスさせる方法を示すcolabノートブックチュートリアルを追加しました。このノートブックはMuJoCoのネイティブPythonバインディングを使用し、Pythonでの簡単なレンダリングのためのドラフト版 ``Renderer`` クラスを含んでいます。
   |br| お試しください：  |LQRopenincolab|

   .. |LQRopenincolab| image:: https://colab.research.google.com/assets/colab-badge.png
                       :target: https://colab.research.google.com/github/deepmind/mujoco/blob/main/python/LQR.ipynb

#. ヒューマノイドモデルの更新：
   - 2つのキーフレーム（片足立ちとスクワット）を追加しました。
   - 最大股関節屈曲角度を増加しました。
   - 股関節屈曲角度が大きい場合に股関節と膝を結合するハムストリングテンドンを追加しました。
   - デフォルトのより良い使用法やより良い命名規則を含む、全般的な外観の改善を行いました。

#. ボックス制約付き二次計画法を解くための :ref:`mju_boxQP` と割り当て関数 :ref:`mju_boxQPmalloc` を追加しました：

   .. math::

      x^* = \text{argmin} \; \tfrac{1}{2} x^T H x + x^T g \quad \text{s.t.} \quad l \le x \le u

   `Tassa et al. 2014 <https://doi.org/10.1109/ICRA.2014.6907001>`__ で紹介されたアルゴリズムは、問題サイズに関係なく2〜5回のコレスキー分解で収束します。

#. 正方行列 :math:`M` を両側のベクトル :math:`x` と :math:`y` で乗算する :ref:`mju_mulVecMatVec` を追加しました。この関数は :math:`x^TMy` を返します。

#. 新しいプラグインAPIを追加しました。プラグインにより、開発者はコアエンジンコードを変更せずにMuJoCoの機能を拡張できます。プラグインメカニズムは既存のコールバックを置き換えることを意図していますが、単純なユースケースや後方互換性のためにコールバックは当面オプションとして残ります。新しいメカニズムは状態を持つプラグインを管理し、異なるソースからの複数のプラグインをサポートするため、MuJoCo拡張機能をグローバルなオーバーライドではなくモジュール方式で導入できます。新しいメカニズムは現在、内部テスト中のため、コード以外にはドキュメント化されていません。プラグインメカニズムの使用に興味がある場合は、まずお問い合わせください。

#. :at:`meshdir` と :at:`texturedir` の両方の値を設定する :at:`assetdir` コンパイラオプションを追加しました。後者の属性の値は :at:`assetdir` よりも優先されます。

#. シミュレーションをより遅い速度で開始するための :at:`realtime` オプションを :ref:`visual<visual>` に追加しました。

#. 新しい :at:`cable` コンポジットタイプを追加しました：

   - ケーブル要素はボールジョイントで接続されます。
   - `initial` パラメータは開始境界のジョイントを指定します： :at:`free` 、 :at:`ball` 、または :at:`none` 。
   - 境界ボディは :at:`B_last` と :at:`B_first` という名前で公開されます。
   - 頂点の初期位置はパラメータ :at:`vertex` でXML内に直接指定できます。
   - ボディフレームの向き **は** カーブのマテリアルフレームの向きです。

#. 新しい :at:`cable` パッシブ力プラグインを追加しました：

   - ねじり剛性と曲げ剛性はパラメータ :at:`twist` と :at:`bend` で個別に設定できます。
   - 応力フリー構成は、フラグ :at:`flat` を使用して初期構成またはフラットに設定できます。
   - プレクトネームの形成を示す新しい `cable.xml <https://github.com/google-deepmind/mujoco/blob/main/model/plugin/elasticity/cable.xml>`__ サンプル。
   - 曲線的な平衡構成を示す新しい `coil.xml <https://github.com/google-deepmind/mujoco/blob/main/model/plugin/elasticity/coil.xml>`__ サンプル。
   - ねじりと異方性の相互作用を示す新しい `belt.xml <https://github.com/google-deepmind/mujoco/blob/main/model/plugin/elasticity/belt.xml>`__ サンプル。
   - カンチレバーの厳密解を使用したテストを追加しました。

   +--------------------------+--------------------------+--------------------------+
   | .. youtube:: 25kQP671fJE | .. youtube:: 4DvGe-BodFU | .. youtube:: QcGdpUd5H0c |
   |   :align: center         |   :align: center         |    :align: center        |
   |   :height: 140px         |   :height: 140px         |    :height: 140px        |
   +--------------------------+--------------------------+--------------------------+

Pythonバインディング
^^^^^^^^^^^^^^^^^^^^^^^
11. `名前付きアクセサ <https://mujoco.readthedocs.io/en/latest/python.html#named-access>`__ オブジェクトに ``id`` と ``name`` プロパティを追加しました。
    これらはそれぞれ ``mj_name2id`` と ``mj_id2name`` へのよりPythonらしいAPIアクセスを提供します。

#. ``MjData.contact`` の長さが ``nconmax`` ではなく ``ncon`` になり、 ``ncon`` をチェックする必要なく直接イテレータとして使用できるようになりました。

#. Python callableがコールバックとしてインストールされた際のメモリリークを修正しました
   （ `#527 <https://github.com/google-deepmind/mujoco/issues/527>`__ ）。


Version 2.2.2 (September 7, 2022)
---------------------------------

全般
^^^^^^^

.. youtube:: BcHZ5BFeTmU
   :aspect: 16:7
   :align: right
   :height: 150px

1. 真空グリッパーや接着性の生体力学的付属器官を模倣する :ref:`接着アクチュエータ<actuator-adhesion>` を追加しました。
#. 関連する `サンプルモデル <https://github.com/google-deepmind/mujoco/tree/main/model/adhesion>`__ と動画を追加しました：
#. サブツリーの重心の並進ヤコビアンを計算する :ref:`mj_jacSubtreeCom` を追加しました。
#. :el:`weld` 制約に :at:`torquescale` と :at:`anchor` 属性を追加しました。 :at:`torquescale` は制約が及ぼすトルク対力の比率を設定し、 :at:`anchor` は溶接レンチが適用される点を設定します。詳細は :ref:`weld<equality-weld>` を参照してください。
#. ``mjNEQDATA`` （ ``mjModel.eq_data`` の等式制約パラメータの行長）を7から11に増加しました。
#. :el:`connect` と :el:`weld` の両方の制約のアンカーポイントの可視化を追加しました（ ``simulate`` の「N」キーで有効化）。
#. 新しいweld属性のさまざまな使用法を示す `weld.xml <https://github.com/google-deepmind/mujoco/blob/main/test/engine/testdata/weld.xml>`__ を追加しました。

   .. youtube:: s-0JHanqV1A
      :aspect: 16:7
      :align: right
      :height: 150px

#. :at:`site` 伝達を持つアクチュエータに参照サイトを追加することで、デカルト6Dエンドエフェクタ制御が可能になりました。 :ref:`アクチュエータ<actuator-general>` ドキュメントの新しい :at:`refsite` 属性の説明と
   `refsite.xml <https://github.com/google-deepmind/mujoco/blob/main/test/engine/testdata/actuation/refsite.xml>`__
   サンプルモデルを参照してください。

#. :at:`autolimits` コンパイラオプションを追加しました。 ``true`` の場合、ジョイントとテンドンの :at:`limited` 属性、およびアクチュエータの :at:`ctrllimited` 、 :at:`forcelimited` 、 :at:`actlimited` 属性は、対応する範囲が *定義されている* 場合は自動的に ``true`` に設定され、そうでない場合は ``false`` に設定されます。

   ``autolimits="false"`` （デフォルト）の場合、 :at:`limited` 属性なしで :at:`range` 属性が指定されたモデルはコンパイルに失敗します。将来のリリースで :at:`autolimits` のデフォルトが ``true`` に変更される予定であり、このコンパイルエラーによりユーザーはこの将来の動作変更を事前に検知できます。

   .. attention::
     これは破壊的変更です。rangeが定義されていたが :at:`limited` が未指定だったモデルでは、現在のモデルの動作を維持するために、limitedを明示的に ``false`` に設定するか、rangeを削除してください。

#. すべての正しく形成されたメッシュに対する慣性モーメント計算を追加しました。このオプションはコンパイラフラグ :at:`exactmeshinertia` を ``true`` に設定することで有効化されます（デフォルトは ``false`` ）。このデフォルトは将来変更される可能性があります。
#. 推論された慣性を境界（シェル）上に配置するためのパラメータ :at:`shellinertia` を :at:`geom` に追加しました。現在はメッシュのみがサポートされています。
#. 体積慣性が推論されるメッシュについて、メッシュ面の向きが一貫していない場合にエラーを発生させるようにしました。これが発生した場合は、MeshLabやBlenderなどでメッシュを修正してください。

   .. youtube:: I2q7D0Vda-A
      :align: right
      :height: 150px

#. 吊り下げテンドンのカテナリー可視化を追加しました。動画に見えるモデルは
   `こちら <https://github.com/google-deepmind/mujoco/blob/main/test/engine/testdata/catenary.xml>`__ にあります。
#. :ref:`visual/global<visual-global>` に ``azimuth`` と ``elevation`` 属性を追加しました。モデルロード時のフリーカメラの初期方向を定義します。
#. 上記の属性を尊重してデフォルトのフリーカメラを設定する ``mjv_defaultFreeCamera`` を追加しました。
#. ``simulate`` がFileセクションのボタンまたは ``Ctrl-P`` によるスクリーンショットの取得をサポートするようになりました。
#. ``simulate`` の時間同期の改善。特に、要求されたファクターと異なる場合（例：タイムステップが非常に小さくシミュレーションがリアルタイムに追いつけない場合）に実際のリアルタイムファクターを報告するようになりました。
#. センサーの無効化フラグを追加しました。
#. :ref:`mju_mulQuat` と :ref:`mju_mulQuatAxis` がインプレース計算をサポートするようになりました。例えば
   |br| ``mju_mulQuat(a, a, b);`` はクォータニオン ``a`` を ``a`` と ``b`` の積に設定します。
#. ``mjd_transitionFD`` にセンサー行列を追加しました（これはAPI変更です）。

削除済み/非推奨の機能
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

21. ``distance`` 制約を削除しました。

バグ修正
^^^^^^^^^

22. 反射内の一部の透明ジオムのレンダリングを修正しました。
#.  ``intvelocity`` のデフォルト解析を修正しました。

Version 2.2.1 (July 18, 2022)
-----------------------------

全般
^^^^^^^

1. 状態遷移行列と制御遷移行列の効率的な有限差分近似を計算する ``mjd_transitionFD`` を追加しました。詳細は :ref:`こちら<derivatives>` を参照してください。
#. 楕円体流体モデルの導関数を追加しました。
#. :ref:`キーフレーム<keyframe>` に ``ctrl`` 属性を追加しました。
#. :ref:`時間を計測する<sensor-clock>` ``clock`` センサーを追加しました。
#. スキンに可視化グループを追加しました。
#. ``free`` および ``ball`` ジョイントと ``site`` 伝達を持つアクチュエータのアクチュエータ可視化を追加しました。
#. アクチュエータの活性化の可視化を追加しました。
#. 「統合速度」アクチュエータのショートカット ``<actuator-intvelocity>`` を追加しました。ドキュメントは :ref:`こちら <actuator-intvelocity>` です。
#. アクティブダンピングアクチュエータのショートカット ``<actuator-damper>`` を追加しました。ドキュメントは :ref:`こちら <actuator-damper>` です。
#. ``mju_rotVecMat`` と ``mju_rotVecMatT`` がインプレース乗算をサポートするようになりました。
#. ``mjData.ctrl`` の値がインプレースでクランプされなくなり、エンジンによって変更されないようになりました。
#. mjDataのバッファ内の配列が、8バイト境界ではなく64バイト境界にアラインされるようになりました。
#. `Address Sanitizer (ASAN) <https://clang.llvm.org/docs/AddressSanitizer.html>`__ および `Memory Sanitizer (MSAN) <https://clang.llvm.org/docs/MemorySanitizer.html>`__ でビルドする際のメモリポイズニングを追加しました。これにより、ASANが ``mjModel.buffer`` と ``mjData.buffer`` 内の配列外の領域への読み書きを検出でき、MSANが ``mj_resetData`` 後の ``mjData`` の未初期化フィールドからの読み取りを検出できるようになります。
#. `スライダークランクのサンプルモデル <https://github.com/google-deepmind/mujoco/tree/main/model/slider_crank>`__ を追加しました。

バグ修正
^^^^^^^^^

15. :ref:`活性化クランプ <CActRange>` が :ref:`陰的積分器<geIntegration>` で適用されていませんでした。
#. 向き指定子の解析を厳格化しました。この変更前は、 ``quat`` と :ref:`代替指定子<COrientation>` の両方を含む指定（例: ``<geom ... quat=".1 .2 .3 .4" euler="10 20 30">``）では ``quat`` が無視され ``euler`` のみが使用されていました。この変更後は解析エラーが発生します。
#. XML属性の解析を厳格化しました。この変更前は ``<geom size="1/2 3 4">`` のような誤ったXMLスニペットが ``size="1 0 0"`` として解析され、エラーは発生しませんでした。現在はエラーが発生します。
#. ``<geom size="1 NaN 4">`` のようにXML経由で ``NaN`` を読み込むことは、デバッグ目的で引き続き許可されますが、警告が表示されるようになりました。
#. ``mj_loadModel`` でのNULLポインタ参照を修正しました。
#. MJBから無効なモデルを読み込む際のメモリリークを修正しました。
#. ``mjModel`` のバッファサイズ計算時に整数オーバーフローが回避されるようになりました。
#. ``mjWARN_BADCTRL`` の欠落していた警告文字列を追加しました。

パッケージング
^^^^^^^^^^^^^^^

23. ``MuJoCo.app`` に埋め込まれた ``mujoco.framework`` のコピーを使用して外部アプリケーションをビルドできるように、MacOSのパッケージングを変更しました。


Version 2.2.0 (May 23, 2022)
----------------------------

オープンソース化
^^^^^^^^^^^^^^^^^

1. MuJoCoが完全なオープンソースソフトウェアになりました。新たに利用可能になったトップレベルディレクトリは以下の通りです：

   a. ``src/``: すべてのソースファイル。サブディレクトリはプログラミングの章の :ref:`概要<inIntro>` に記載されているモジュールに対応しています：

   - ``src/engine/``: コアエンジン。
   - ``src/xml/``: XMLパーサー。
   - ``src/user/``: モデルコンパイラ。
   - ``src/visualize/``: 抽象ビジュアライザー。
   - ``src/ui/``: UIフレームワーク。

   b. ``test/``: テストおよび対応するアセットファイル。

   c. ``dist/``: パッケージングとバイナリ配布に関連するファイル。

#. `コントリビューターガイド <https://github.com/google-deepmind/mujoco/blob/main/CONTRIBUTING.md>`__ と `スタイルガイド <https://github.com/google-deepmind/mujoco/blob/main/STYLEGUIDE.md>`__ を追加しました。

全般
^^^^^^^

3. 速度に関する滑らかな（非制約）動力学の力の解析的導関数を追加しました：

   - Recursive Newton-Eulerアルゴリズムで計算される遠心力とコリオリ力。
   - 減衰と流体抵抗の受動力。
   - アクチュエーション力。

#. ``implicit`` 積分器を追加しました。上記の解析的導関数を使用して、新しい速度に対する陰的積分器が追加されました。この積分器は、安定性と計算コストの両面でオイラー積分器とルンゲ・クッタ積分器の中間に位置します。流体抵抗を使用するモデル（例：飛行や水泳）や :ref:`速度アクチュエータ<actuator-velocity>` を使用するモデルに最も有用です。詳細は :ref:`数値積分<geIntegration>` セクションを参照してください。

#. :ref:`一般アクチュエータ<actuator-general>` に :at:`actlimited` と :at:`actrange` 属性を追加しました。アクチュエータの内部状態（活性化）をクランプするためのものです。このクランプは統合速度アクチュエータに有用です。詳細は :ref:`活性化クランプ <CActRange>` セクションを参照してください。

#. ``mjData`` のフィールド ``qfrc_unc`` （非制約力）と ``qacc_unc`` （非制約加速度）をそれぞれ ``qfrc_smooth`` と ``qacc_smooth`` に名前変更しました。「unconstrained」は正確ですが、「smooth」の方が「unc」よりもわかりやすいためです。

#. パブリックヘッダーが ``/include`` から ``/include/mujoco/`` に移動されました。これは他のオープンソースプロジェクトで一般的なディレクトリレイアウトに沿ったものです。開発者は自身のコードベースでMuJoCoのパブリックヘッダーを ``#include <mujoco/filename.h>`` を使用してインクルードすることが推奨されます。

#. :ref:`shadowsize<visual-quality>` 属性で指定されるデフォルトのシャドウ解像度が1024から4096に増加されました。

#. 保存されたXMLで2スペースのインデントが使用されるようになりました。

バグ修正
^^^^^^^^^

10. セグメンテーションレンダリングでアンチエイリアシングが無効化されました。この変更前は、 :ref:`offsamples<visual-quality>` 属性が0より大きい場合（デフォルト値は4）、複数のジオムにまたがるピクセルが平均化されたセグメンテーションIDを受け取り、不正確または存在しないIDが生じていました。この変更後、セグメンテーションレンダリング中は :at:`offsamples` が無視されます。

#.  実験的なmultiCCD機能の有効化フラグの値が他の有効化フラグと連続するようにしました。連続性は ``simulate`` UIおよびその他の箇所で想定されています。

#.  ``mj_saveLastXML`` を使用してOBJメッシュを含むモデルを保存する際のメッシュ重複の問題を修正しました。


Version 2.1.5 (Apr. 13, 2022)
-----------------------------

全般
^^^^^^^

1. 実験的機能として、有効化フラグで活性化されるマルチコンタクト凸衝突検出を追加しました。詳細な説明は :ref:`こちら <option-flag>` を参照してください。

バグ修正
^^^^^^^^^

2. Linux上のGLAD初期化ロジックが、 ``*GetProcAddress`` 関数がプロセスのグローバルシンボルテーブルに存在しない場合に ``dlopen`` を呼び出してGLプラットフォーム動的ライブラリをロードするようになりました。特に、GLFWを使用してレンダリングコンテキストを設定するプロセスで ``libGLX.so`` に明示的にリンクされていない場合（これはPythonインタプリタなどに該当します）、 ``mjr_makeContext`` が呼び出されたときに ``gladLoadGL`` エラーで失敗するのではなく、正しく動作するようになりました。

#. Pythonバインディングで、スカラーフィールドの名前付きインデクサー（例：アクチュエータの ``ctrl`` フィールド）が ``()`` ではなく ``(1,)`` の形状のNumPy配列を返すようになりました。これにより、これらのフィールドへの値の代入がより簡単になります。

Version 2.1.4 (Apr. 4, 2022)
----------------------------

全般
^^^^^^^

1. MuJoCoがGLEWの代わりにGLADを使用してOpenGL APIアクセスを管理するようになりました。Linux上では、GLX、EGL、またはOSMesaのどれを使用しているかに応じて異なるGLラッピングライブラリにリンクする必要がなくなりました。代わりに、ユーザーはGLX、EGL、またはOSMesaを使用してGLコンテキストを作成するだけで、 ``mjr_makeContext`` が使用されているものを検出します。

#. コンタクトフレームの可視化を追加しました。これは衝突関数を作成または修正する際に、コンタクトのx軸とy軸の実際の方向が重要な場合に有用です。

バイナリビルド
^^^^^^^^^^^^^^^

3. LinuxおよびWindowsで ``_nogl`` 動的ライブラリが提供されなくなりました。GLADへの切り替えにより、ライブラリのロード時ではなく ``mjr_makeContext`` が呼び出された時にOpenGLシンボルを解決できるようになりました。その結果、MuJoCoライブラリはOpenGLへの明示的な動的依存関係を持たなくなり、OpenGLが存在しないシステムでも使用できるようになりました。

Simulate
^^^^^^^^

4. モデルがロードされていない状態で '[' または ']' を押すとクラッシュする ``simulate`` のバグを修正しました。

#. Simulate GUIにコンタクトフレームの可視化が追加されました。

#. "set key"、"reset to key" をそれぞれ "save key" と "load key" に名前変更しました。

#. F6とF7のキーバインドを、あまり有用でない「垂直同期」と「ビジーウェイト」から、より有用なフレームとラベルのサイクリングに変更しました。

バグ修正
^^^^^^^^^

8. ``mj_resetData`` が ``solver_nnz`` フィールドをゼロにするようにしました。

#. ``mju_quat2mat`` の単位クォータニオンに対する特殊分岐を削除しました。以前は、 ``mju_quat2mat`` はクォータニオンの実部が1.0に等しい場合にすべての計算をスキップしていました。非常に小さい角度の場合（例：有限差分時）、コサインがdouble精度でちょうど1.0に評価される一方、サインはまだ非ゼロである可能性があります。


Version 2.1.3 (Mar. 23, 2022)
-----------------------------

全般
^^^^^^^

1. ``simulate`` が（ ``[`` と ``]`` キーによる）カメラの切り替えをサポートするようになりました。
#. ``mjVIS_STATIC`` がワールドの直接の子だけでなく、すべての静的ボディを切り替えるようになりました。

Pythonバインディング
^^^^^^^^^^^^^^^^^^^^^

3. ``MjrContext`` に ``free()`` メソッドを追加しました。
#. 列挙型が数値との算術演算とビット演算をサポートするようになりました。

バグ修正
^^^^^^^^^

5. 2.1.2で導入された平面のレンダリングバグを修正しました。これは `dm_control <https://github.com/google-deepmind/dm_control>`__ の迷路環境を破壊していました。


Version 2.1.2 (Mar. 15, 2022)
-----------------------------

新規モジュール
^^^^^^^^^^^^^^^

1. 新しい :doc:`Pythonバインディング<python>` を追加しました。 ``pip install mujoco`` でインストールでき、 ``import mujoco`` でインポートできます。
#. 新しい :doc:`Unityプラグイン<unity>` を追加しました。
#. 新しい ``introspect`` モジュールを追加しました。これはMuJoCoのパブリックAPIに対するリフレクションのような機能を提供し、現在は関数と列挙型を記述しています。Pythonで実装されていますが、このモジュールは複数の言語を対象とする自動コード生成に広く有用であることが期待されます。（これは ``mujoco`` Pythonバインディングパッケージの一部としては出荷されません。）

API変更
^^^^^^^^^

4. 浮動小数点型 ``mjtNum`` の定義を新しいヘッダー `mjtnum.h <https://github.com/google-deepmind/mujoco/blob/3577e2cf8bf841475b489aefff52276a39f24d51/include/mjtnum.h>`__ に移動しました。
#. ヘッダー `mujoco_export.h` を :ref:`mjexport.h<inHeader>` に名前変更しました。
#. 浮動小数点数のフォーマット文字列を受け取る ``mj_printFormattedData`` を追加しました。例えば精度を上げるために使用できます。

全般
^^^^^^^

7. MuJoCoが `OBJ <https://en.wikipedia.org/wiki/Wavefront_.obj_file>`__ メッシュファイルを読み込めるようになりました。

   a. 4頂点を超える多角形を含むメッシュはサポートされていません。
   #. 複数のオブジェクトグループを含むOBJファイルでは、最初のグループ以降のグループは無視されます。
   #. （リリース後に追加、2.1.2アーカイブには含まれていません）テクスチャ付きの `マグカップ <https://github.com/google-deepmind/mujoco/blob/main/model/mug/mug.xml>`__ サンプルモデルを追加しました：

      .. image:: images/changelog/mug.png
         :width: 300px


#. :ref:`framepos<sensor-framepos>` 、 :ref:`framequat<sensor-framequat>` 、 :ref:`framexaxis<sensor-framexaxis>` 、 :ref:`frameyaxis<sensor-frameyaxis>` 、 :ref:`framezaxis<sensor-framezaxis>` 、 :ref:`framelinvel<sensor-framelinvel>` 、および :ref:`frameangvel<sensor-frameangvel>` センサーにオプションの参照フレーム指定を追加しました。参照フレームは新しい :at:`reftype` と :at:`refname` 属性で指定されます。

#. :ref:`ユーザーパラメータ <CUser>` のサイズが自動推論されるようになりました。

   a. トップレベルの :ref:`size <size>` 句でのユーザーパラメータの宣言（例: :at:`nuser_body` 、 :at:`nuser_jnt` 等）が-1の値を受け付けるようになりました（デフォルト値）。これにより、モデルで定義された対応する :at:`user` 属性の最大長に自動的に設定されます。
   #. -1未満の値を設定するとコンパイラエラーが発生します（以前はセグメンテーション違反でした）。
   #. モデルで定義された一部の :at:`user` 属性より短い値を設定するとエラーが発生します（以前は追加の値が無視されていました）。

#. :ref:`mjvScene` 内のライトの最大数が8から100に増加されました。

#. 保存されたXMLファイルは、元のXMLに明示的な :ref:`inertial <body-inertial>` 要素が含まれている場合にのみ、それを含むようになりました。コンパイラの :ref:`inertiafromgeom <compiler>` メカニズムによって自動推論された慣性は未指定のままになります。

#. ユーザーが選択したジオムは常に不透明としてレンダリングされるようになりました。これはインタラクティブなビジュアライザーで有用です。

#. 静的ジオムが可視化のために :ref:`ジオムグループ<body-geom>` を尊重するようになりました。この変更まで、静的ジオムのレンダリングは :ref:`mjVIS_STATIC<mjtVisFlag>` 可視化フラグを使用してのみ切り替えられました。この変更後は、ジオムがレンダリングされるためにはジオムグループと可視化フラグの両方が有効である必要があります。

#. :ref:`mujoco.h<inHeader>` の関数宣言におけるポインタパラメータで、固定長配列を表すものが、例えば ``mjtNum* quat`` ではなく ``mjtNum quat[4]`` のように範囲付きの配列として記述されるようになりました。CおよびC++の観点からは、関数シグネチャ内の配列型はポインタ型に退化するため、これは変更ではありません。ただし、自動生成されたコードが期待される入力形状を認識できるようになります。

#. 実験的なステートレス流体相互作用モデル。 :ref:`こちら <gePassive>` に記載されているように、流体力はボディの慣性から計算されたサイズを使用します。これは便利な場合もありますが、良い近似であることは非常にまれです。新しいモデルでは、力はボディではなくジオムに作用し、いくつかのユーザー設定可能なパラメータがあります。このモデルは新しい属性 ``<geom fluidshape="ellipsoid"/>`` を設定することで有効化されます。パラメータは :ref:`こちら<body-geom>` に簡潔に記載されていますが、モデルとそのパラメータの完全な説明は、この機能が実験的ステータスを脱した時点に委ねます。

バグ修正
^^^^^^^^^

16. ``mj_loadXML`` と ``mj_saveLastXML`` がロケール非依存になりました。コンマを小数点区切りとして使用するシステムロケールのユーザーでも、Unityプラグインが正しく動作するようになりました。
#.  VFS内のXMLアセットがNULL文字で終わる必要がなくなりました。代わりに、ファイルサイズは対応するVFSエントリのサイズパラメータによって決定されます。
#.  スキンを使用する際の ``mjrContext`` の頂点バッファオブジェクトのメモリリークを修正しました。
#.  カメラのクォータニオンがXMLコンパイル時に正規化されるようになりました。

バイナリビルド
^^^^^^^^^^^^^^^

20. WindowsバイナリがClangでビルドされるようになりました。

Version 2.1.1 (Dec. 16, 2021)
-----------------------------

API変更
^^^^^^^^^

1. 浮動小数点数のフォーマット文字列を受け取る ``mj_printFormattedModel`` を追加しました。例えば精度を上げるために使用できます。
#. MuJoCoバイナリのバージョンを表す人間が読める文字列を返す ``mj_versionString`` を追加しました。
#. API構造体定義のプライベートインスタンスにおける先頭のアンダースコアを末尾のアンダースコアに変換しました。予約済み識別子の指令に準拠するためです。 `C標準: セクション7.1.3 <https://www.open-std.org/jtc1/sc22/wg14/www/docs/n1570.pdf>`__ を参照してください。

   .. attention::
      これは軽微な破壊的変更です。プライベートインスタンスを参照するコードは動作しなくなります。修正するには、先頭のアンダースコアを末尾のアンダースコアに置換してください（例: ``_mjModel`` |rarr| ``mjModel_``）。

全般
^^^^^^^

4. より安全な文字列処理: ``strcat`` 、 ``strcpy`` 、 ``sprintf`` をそれぞれ ``strncat`` 、 ``strncpy`` 、 ``snprintf`` に置き換えました。
#. インデントを4スペースから2スペースに変更し、K&R中括弧スタイルを採用し、一行の条件文に中括弧を追加しました。

バグ修正
^^^^^^^^^

6. PGSソルバーでの未初期化メモリからの読み取りを修正しました。
#. 計算されたカプセルの慣性が正確になりました。この変更まで、 :ref:`コンパイラ <compiler>` の :at:`inertiafromgeom` メカニズムによって計算されるカプセルの質量と慣性は、カプセルの円筒形の中央部分を両端でカプセル半径の半分だけ延長した円筒で近似されていました。カプセルの慣性は `平行軸の定理 <https://en.wikipedia.org/wiki/Parallel_axis_theorem>`__ を2つの半球形のエンドキャップに適用して計算されるようになりました。

   .. attention::
      これは軽微な破壊的変更です。自動計算されたカプセルの慣性を持つモデルのシミュレーションは数値的に異なるものになり、例えばゴールデンバリューテストの破壊につながります。
#. :ref:`force <sensor-force>` と :ref:`torque <sensor-torque>` センサーに関連するバグを修正しました。この変更まで、F/Tセンサーが報告する力とトルクは、接触によって生成されたものを除き、ツリー外の制約レンチを無視していました。forceとtorqueセンサーは :ref:`connect <equality-connect>` と :ref:`weld <equality-weld>` 制約の効果を正しく考慮するようになりました。

   .. note::
      キネマティックツリーの外側（つまり、祖先関係のないボディ間）にある :ref:`空間テンドン <tendon-spatial>` によって生成される力は、forceとtorqueセンサーではまだ考慮されていません。これは今後の作業項目として残っています。

コードサンプル
^^^^^^^^^^^^^^^

9. ``testspeed``: デフォルトで有効な疑似ランダム制御ノイズの注入を追加しました。これは何らかの固定された接触構成に落ち着いて非現実的な計測結果を提供することを避けるためです。
#. ``simulate``:

   a. '+' と '-' キーで制御される、リアルタイムより遅い速度の機能を追加しました。
   #. 制御にブラウン運動ノイズを注入するためのスライダーを追加しました。
   #. 現在のカメラのポーズを含むMJCF句を出力する "Print Camera" ボタンを追加しました。
   #. 同じモデルファイルを再読み込みする際にカメラのポーズがリセットされなくなりました。

依存関係の更新
^^^^^^^^^^^^^^^

11. ``TinyXML`` が ``TinyXML2`` 6.2.0に置き換えられました。
#. ``qhull`` がバージョン8.0.2にアップグレードされました。
#. ``libCCD`` がバージョン1.4にアップグレードされました。
#. Linuxで ``libstdc++`` が ``libc++`` に置き換えられました。

バイナリビルド
^^^^^^^^^^^^^^^

15. MacOSパッケージング。Apple SiliconとIntel CPUの両方をネイティブにサポートするUniversalバイナリを出荷するようになりました。

    a. MuJoCoライブラリが `Framework Bundle
       <https://developer.apple.com/library/archive/documentation/MacOSX/Conceptual/BPFrameworks/Concepts/FrameworkAnato
       my.html>`__ としてパッケージ化されるようになり、Xcodeプロジェクト（Swiftプロジェクトを含む）に組み込みやすくなりました。開発者は ``-framework mujoco`` フラグを使用してMuJoCoのコンパイルとリンクを行うことが推奨されますが、すべてのヘッダーファイルと ``libmujoco.2.1.1.dylib`` ライブラリにはフレームワーク内部から直接アクセスすることも可能です。
    #. サンプルアプリケーションが ``MuJoCo.app`` というApplication Bundleにパッケージ化されるようになりました。GUIから起動すると、バンドルは ``simulate`` 実行可能ファイルを起動します。その他のプリコンパイルされたサンプルプログラムはバンドル内（ ``MuJoCo.app/Contents/MacOS`` ）に同梱されており、コマンドラインから起動できます。
    #. バイナリが署名され、ディスクイメージが公証されるようになりました。

#. Windowsバイナリとライブラリが署名されるようになりました。
#. LinuxとmacOSでリンク時最適化が有効化され、3つのテストモデル（ ``cloth.xml`` 、 ``humanoid.xml`` 、 ``humanoid100.xml`` ）でベンチマークした場合に平均で約20%のスピードアップが得られました。
#. LinuxバイナリがGCCの代わりにLLVM/Clangでビルドされるようになりました。
#. AArch64（別名ARM64）Linux向けビルドも提供されるようになりました。
#. LinuxとMacOSの共有ライブラリからプライベートシンボルが削除されなくなりました。

サンプルモデル
^^^^^^^^^^^^^^^
21. ``model/`` ディレクトリのクリーンアップ。

    a. サブディレクトリに再編成し、すべての依存関係を含めるようにしました。
    #. XMLコメントに説明を追加し、XMLをクリーンアップしました。
    #. いくつかのcompositeモデルを削除しました: ``grid1`` 、 ``grid1pin`` 、 ``grid2`` 、 ``softcylinder`` 、 ``softellipsoid`` 。

#. ``docs/images/models/`` に説明用アニメーションを追加しました：

|humanoid|   |particle|


Version 2.1.0 (Oct. 18, 2021)
-----------------------------

新機能
^^^^^^^^

1. キーフレームに ``mocap_pos`` と ``mocap_quat`` フィールド（XMLではmposとquat属性）が追加され、mocapポーズをキーフレームに保存できるようになりました。
2. 新しいユーティリティ関数: ``mju_insertionSortInt`` （整数の挿入ソート）と ``mju_sigmoid`` （2つの半二次関数からシグモイドを構成）。

全般
^^^^^^^

3. 仮想ファイルシステム（VFS）の事前確保サイズが2000と1000に増加され、より大規模なプロジェクトに対応できるようになりました。
#. ``mjuiItem`` 共用体内のC構造体に名前が付けられ、互換性が向上しました。
#. 修正: ``mjcb_contactfilter`` の型が ``mjfConFilt`` になりました（以前は ``mjfGeneric`` でした）。
#. 修正: ``mjCModel`` のセンサー配列がクリアされていませんでした。
#. クロスプラットフォームコードをクリーンアップしました（内部変更、APIからは見えません）。
#. XMLの ``texcoord`` データの解析に関するバグを修正しました（頂点数に関連）。
#. ``nkey`` （キーフレーム数）に関連する `simulate.cc <https://github.com/google-deepmind/mujoco/blob/main/simulate/simulate.cc>`__ のバグを修正しました。
#. 非衝突ジオム（ ``contype==0 and conaffinity==0`` ）が多数存在する場合の衝突検出を高速化しました。

UI
^^

11. フィギュアの選択型が ``int`` から ``float`` に変更されました。
#. 選択とハイライトが有効な場合に、フィギュアがデータ座標を表示するようになりました。
#. ``mjMAXUIMULTI`` を35、 ``mjMAXUITEXT`` を300、 ``mjMAXUIRECT`` を25に変更しました。
#. 折りたたみ可能なサブセクションを追加しました。状態付きのセパレータとして実装されており、 ``mjSEPCLOSED`` が折りたたみ状態、 ``mjSEPCLOSED+1`` が展開状態です。
#. ``mjITEM_RADIOLINE`` アイテムタイプを追加しました。
#. UIセクションの構築を簡素化する ``mjui_addToSection`` 関数を追加しました。
#. ``mjvFigure`` にサブプロットタイトルを追加しました。

レンダリング
^^^^^^^^^^^^^

18. ``render_gl2`` が軸範囲計算における非有限浮動小数点データを防御するようになりました。
#. ``render_gl2`` が可視性向上のために線を後ろから前に描画するようになりました。
#. ``mjr_label`` 関数（テキストラベル用）を追加しました。
#. ``mjr_render`` が ``ngeom==0`` の場合に即座に終了するようになり、未初期化のシーン（例: ``frustrum==0``）によるエラーを回避します。
#. ``mjr_render`` にシザーボックスを追加し、毎フレームウィンドウ全体をクリアしないようにしました。


ライセンスマネージャー
^^^^^^^^^^^^^^^^^^^^^^^

23. ライセンスマネージャー全体を削除しました。関数 ``mj_activate`` と ``mj_deactivate`` は後方互換性のために残されていますが、何も実行しなくなり、呼び出す必要もなくなりました。
#. リモートライセンス証明書関数 ``mj_certXXX`` を削除しました。

以前のバージョン
-----------------

以前のバージョンの変更ログについては `roboti.us <https://www.roboti.us/download.html>`__ を参照してください。

.. |humanoid| image:: images/models/humanoid.gif
   :width: 270px
.. |particle| image:: images/models/particle.gif
   :width: 270px
