.. _Parseandcompile:

解析とコンパイル
^^^^^^^^^^^^^^^^^^^^^^^^

ここでの主要な関数は :ref:`mj_loadXML` です。この関数は組み込みのパーサーとコンパイラを呼び出し、有効な
mjModel へのポインタを返すか、NULL を返します。NULL の場合、ユーザーはユーザー提供の文字列でエラー情報を
確認する必要があります。モデルおよびモデルが参照するすべてのファイルは、ディスクから、または提供された場合は
VFSから読み込むことができます。

.. _mj_loadXML:

`mj_loadXML <#mj_loadXML>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_loadXML

MJCFまたはURDF形式のXMLファイルを解析し、コンパイルして低レベルモデルを返す。

vfs が NULL でない場合、ディスクから読み込む前にvfs内のファイルを検索する。

error が NULL でない場合、サイズ error_sz を持つ必要がある。

*Nullable:* ``vfs``, ``error``

.. _mj_parseXML:

`mj_parseXML <#mj_parseXML>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_parseXML

XMLファイルからspecを解析する。

*Nullable:* ``vfs``, ``error``

.. _mj_parseXMLString:

`mj_parseXMLString <#mj_parseXMLString>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_parseXMLString

XML文字列からspecを解析する。

*Nullable:* ``vfs``, ``error``

.. _mj_parse:

`mj_parse <#mj_parse>`__
~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_parse

ファイルからspecを解析する。

*Nullable:* ``vfs``, ``error``

.. _mj_compile:

`mj_compile <#mj_compile>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_compile

:ref:`mjSpec` を :ref:`mjModel` にコンパイルする。specは複数回編集・コンパイルでき、編集内容を反映した
新しい :ref:`mjModel` インスタンスを返す。
コンパイルに失敗した場合、 :ref:`mj_compile` は ``NULL`` を返す。エラーは :ref:`mjs_getError` で読み取れる。

.. _mj_copyBack:

`mj_copyBack <#mj_copyBack>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_copyBack

実数値配列をmodelからspecにコピーする。成功した場合1を返す。

.. _mj_recompile:

`mj_recompile <#mj_recompile>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_recompile

状態を保持しながらspecをmodelに再コンパイルする。 :ref:`mj_compile` と同様に、この関数は :ref:`mjSpec` を
:ref:`mjModel` にコンパイルするが、2つの違いがある。第一に、まったく新しいモデルを返すのではなく、既存の
:ref:`mjModel` と :ref:`mjData` のインスタンスをインプレースで再割り当てする。第二に、提供された :ref:`mjData`
インスタンスの :ref:`積分状態<siIntegrationState>` を保持しつつ、新たに追加または削除された自由度を考慮する。
これにより、ユーザーはモデルをプログラムで編集しながら、同じmodelおよびdata構造体ポインタでシミュレーションを
続行できる。

:ref:`mj_recompile` はコンパイルが成功した場合0を返す。失敗した場合、指定された :ref:`mjModel` と :ref:`mjData`
インスタンスは削除される。 :ref:`mj_compile` と同様に、コンパイルエラーは :ref:`mjs_getError` で読み取れる。

.. _mj_saveLastXML:

`mj_saveLastXML <#mj_saveLastXML>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_saveLastXML

:ref:`mj_loadXML` で作成された低レベルモデルの情報でXMLデータ構造を更新し、MJCFとして保存する。
error が NULL でない場合、サイズ error_sz を持つ必要がある。

この関数は :ref:`mj_loadXML` （レガシーな読み込みメカニズム）で読み込まれたモデルのみを保存することに注意。
旧方式と新方式のモデル読み込み・保存メカニズムの違いについては、 :ref:`モデル編集<meOverview>` の章を参照。

.. _mj_freeLastXML:

`mj_freeLastXML <#mj_freeLastXML>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_freeLastXML

読み込まれた最後のXMLモデルを解放する。各読み込み時に内部的に呼び出される。

.. _mj_saveXMLString:

`mj_saveXMLString <#mj_saveXMLString>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_saveXMLString

specをXML文字列に保存する。成功時に0、失敗時に-1を返す。出力バッファの長さが不足している場合、必要なサイズを
返す。XML保存時にspecは自動的にコンパイルされる。

.. _mj_saveXML:

`mj_saveXML <#mj_saveXML>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_saveXML

specをXMLファイルに保存する。成功時に0、それ以外は-1を返す。XML保存にはspecが先にコンパイルされている
必要がある。

.. _mju_getXMLDependencies:

`mju_getXMLDependencies <#mju_getXMLDependencies>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_getXMLDependencies

MJCFファイル名を指定して、依存するすべてのアセットファイルのリストをdependenciesに格納する。

検索は再帰的に行われ、リストにはファイル名自体も含まれる。

.. _Mainsimulation:

メインシミュレーション
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

これらはシミュレータへの主要なエントリーポイントである。ほとんどのユーザーは :ref:`mj_step` を呼び出すだけで
よく、この関数はすべてを計算してシミュレーション状態を1タイムステップ進める。制御と印加力は事前に
（ ``mjData.{ctrl, qfrc_applied, xfrc_applied}`` に）設定するか、制御と印加力が必要になる直前に呼び出される
制御コールバック :ref:`mjcb_control` をインストールする必要がある。あるいは、 :ref:`mj_step1` と
:ref:`mj_step2` を使用して、シミュレーションパイプラインを制御が必要になる前後の計算に分割することもできる。
この方法では、 :ref:`mj_step1` の結果に依存する制御を設定できる。ただし、RK4ソルバーは
mj_step1/2では動作しないことに注意。詳細は :ref:`Pipeline` を参照。

mj_forward は :ref:`mj_step` と同じ計算を行うが、積分は行わない。モデルの読み込みまたはリセット後に
（mjData全体を有効な状態にするために）有用であり、サンプリングや有限差分近似を含む順序外の計算にも有用である。

:ref:`mj_inverse` は逆動力学を実行し、出力を ``mjData.qfrc_inverse`` に書き込む。この関数を呼び出す前に
``mjData.qacc`` を設定する必要があることに注意。状態（qpos, qvel, act）が与えられた場合、mj_forward は力から
加速度へ、mj_inverse は加速度から力へマッピングする。数学的にはこれらの関数は互いに逆であるが、順動力学は
通常早期に終了する制約最適化アルゴリズムに依存するため、数値的には常にそうとは限らない。順動力学と逆動力学の
結果の差は :ref:`mj_compareFwdInv` 関数で計算でき、これはソルバーの精度チェック（および一般的な整合性チェック）
として利用できる。

:ref:`mj_forward` と :ref:`mj_inverse` のskipバージョンは、例えばqposは変更されずqvelのみが変更された場合
（通常は有限差分の文脈で）に有用である。その場合、qposのみに依存する計算を繰り返す必要はない。
skipstage = :ref:`mjSTAGE_POS<mjtStage>` でダイナミクスを呼び出すことで、これらの節約が実現される。

.. _mj_step:

`mj_step <#mj_step>`__
~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_step

シミュレーションを進める。制御コールバックを使用して外力と制御を取得する。

.. _mj_step1:

`mj_step1 <#mj_step1>`__
~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_step1

シミュレーションを2段階で進める: ユーザーによる外力と制御の設定前。

.. _mj_step2:

`mj_step2 <#mj_step2>`__
~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_step2

シミュレーションを2段階で進める: ユーザーによる外力と制御の設定後。

.. _mj_forward:

`mj_forward <#mj_forward>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_forward

順動力学: mj_stepと同じだが時間積分を行わない。

.. _mj_inverse:

`mj_inverse <#mj_inverse>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_inverse

逆動力学: 呼び出し前にqaccを設定する必要がある。

.. _mj_forwardSkip:

`mj_forwardSkip <#mj_forwardSkip>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_forwardSkip

スキップ付き順動力学。skipstageはmjtStage。

.. _mj_inverseSkip:

`mj_inverseSkip <#mj_inverseSkip>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_inverseSkip

スキップ付き逆動力学。skipstageはmjtStage。

.. _Support:

サポート
^^^^^^^^^^^^

これらはユーティリティ関数とは異なり、 :ref:`mjModel` と :ref:`mjData` へのアクセスを必要とするサポート関数です。
サポート関数はシミュレータ内で呼び出されますが、一部はカスタム計算にも有用であり、以下で詳しく説明します。

.. _mj_stateSize:

`mj_stateSize <#mj_stateSize>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_stateSize

指定された状態シグネチャに必要な :ref:`mjtNum` の数を返します。整数 ``sig`` のビットは :ref:`mjtState` の要素フィールドに対応します。

.. _mj_getState:

`mj_getState <#mj_getState>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_getState

``sig`` で指定された連結状態コンポーネントを ``d`` から ``state`` にコピーします。整数 ``sig`` のビットは :ref:`mjtState` の要素フィールドに対応します。 ``sig`` が無効な場合、 :ref:`mju_error` で失敗します。

.. _mj_extractState:

`mj_extractState <#mj_extractState>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_extractState

``srcsig`` で指定されたコンポーネントを持つ :ref:`mj_getState` で以前取得した状態 ``src`` から、 ``dstsig`` で指定されたコンポーネントのサブセットを ``dst`` に抽出します。 ``dstsig`` で設定されたビットが ``srcsig`` で設定されたビットのサブセットでない場合、 :ref:`mju_error` で失敗します。

.. _mj_setState:

`mj_setState <#mj_setState>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_setState

``sig`` で指定された連結状態コンポーネントを ``state`` から ``d`` にコピーします。整数 ``sig`` のビットは :ref:`mjtState` の要素フィールドに対応します。 ``sig`` が無効な場合、 :ref:`mju_error` で失敗します。

.. _mj_copyState:

`mj_copyState <#mj_copyState>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_copyState

srcからdstに状態をコピーします。

.. _mj_setKeyframe:

`mj_setKeyframe <#mj_setKeyframe>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_setKeyframe

現在の状態をk番目のモデルキーフレームにコピーします。

.. _mj_addContact:

`mj_addContact <#mj_addContact>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_addContact

d->contactリストに接触を追加します。成功した場合は0を返し、バッファが満杯の場合は1を返します。

.. _mj_isPyramidal:

`mj_isPyramidal <#mj_isPyramidal>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_isPyramidal

摩擦円錐のタイプを判定します。

.. _mj_isSparse:

`mj_isSparse <#mj_isSparse>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_isSparse

制約ヤコビアンのタイプを判定します。

.. _mj_isDual:

`mj_isDual <#mj_isDual>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_isDual

ソルバーのタイプを判定します（PGSは双対、CGとニュートンは主問題）。

.. _mj_mulJacVec:

`mj_mulJacVec <#mj_mulJacVec>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_mulJacVec

この関数は制約ヤコビアン mjData.efc_J にベクトルを乗算します。ヤコビアンは密行列またはスパース行列のいずれかであり、この関数はその設定を認識しています。Jによる乗算は、関節空間から制約空間への速度のマッピングを行います。

.. _mj_mulJacTVec:

`mj_mulJacTVec <#mj_mulJacTVec>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_mulJacTVec

mj_mulJacVecと同じですが、ヤコビアンの転置行列を乗算します。これは制約空間から関節空間への力のマッピングを行います。

.. _mj_jac:

`mj_jac <#mj_jac>`__
~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_jac

この関数はエンドエフェクタの運動学的ヤコビアンを計算し、自由度と指定された点の間の局所的な線形関係を記述します。整数ID（ ``body`` ）で指定されたボディと、ボディに取り付けられているものとして扱われるワールドフレーム内の3D点（ ``point`` ）が与えられた場合、ヤコビアンは並進（ ``jacp`` ）と回転（ ``jacr`` ）の両方のコンポーネントを持ちます。いずれかのポインタに ``NULL`` を渡すと、その部分の計算はスキップされます。各コンポーネントは3×nvの行列です。この行列の各行は、指定された点の対応する座標の自由度に関する勾配です。ヤコビアンが計算されるフレームは、ボディの重心を中心としますが、ワールドフレームに整列しています。ヤコビアン計算が現在の一般化座標 ``mjData.qpos`` と整合するために必要な最小の :ref:`パイプラインステージ<piForward>` は、 :ref:`mj_kinematics` の後に :ref:`mj_comPos` を実行することです。

*Nullable:* ``jacp``, ``jacr``

.. _mj_jacBody:

`mj_jacBody <#mj_jacBody>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_jacBody

この関数と残りのヤコビアン関数のバリエーションは、ボディ、ジオム、またはサイトの中心を使って内部的にmj_jacを呼び出します。これらは単なるショートカットであり、mj_jacを直接呼び出すことで同じ結果を得られます。

*Nullable:* ``jacp``, ``jacr``

.. _mj_jacBodyCom:

`mj_jacBodyCom <#mj_jacBodyCom>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_jacBodyCom

ボディの重心エンドエフェクタヤコビアンを計算します。

*Nullable:* ``jacp``, ``jacr``

.. _mj_jacSubtreeCom:

`mj_jacSubtreeCom <#mj_jacSubtreeCom>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_jacSubtreeCom

サブツリーの重心エンドエフェクタヤコビアンを計算します。

.. _mj_jacGeom:

`mj_jacGeom <#mj_jacGeom>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_jacGeom

ジオムのエンドエフェクタヤコビアンを計算します。

*Nullable:* ``jacp``, ``jacr``

.. _mj_jacSite:

`mj_jacSite <#mj_jacSite>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_jacSite

サイトのエンドエフェクタヤコビアンを計算します。

*Nullable:* ``jacp``, ``jacr``

.. _mj_jacPointAxis:

`mj_jacPointAxis <#mj_jacPointAxis>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_jacPointAxis

点の並進エンドエフェクタヤコビアンと軸の回転ヤコビアンを計算します。

*Nullable:* ``jacPoint``, ``jacAxis``

.. _mj_jacDot:

`mj_jacDot <#mj_jacDot>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_jacDot

この関数は :ref:`mj_jac` で計算されたエンドエフェクタの運動学的ヤコビアンの時間微分を計算します。
計算が現在の一般化座標と速度 ``mjData.{qpos, qvel}`` と整合するために必要な最小の :ref:`パイプラインステージ<piStages>` は、 :ref:`mj_kinematics` 、 :ref:`mj_comPos` 、 :ref:`mj_comVel` （この順序）です。

*Nullable:* ``jacp``, ``jacr``

.. _mj_angmomMat:

`mj_angmomMat <#mj_angmomMat>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_angmomMat

この関数は ``3 x nv`` の角運動量行列 :math:`H(q)` を計算し、一般化速度からサブツリー角運動量への線形マッピングを提供します。より正確には、 :math:`h` が ``mjData.subtree_angmom`` 内のボディインデックス ``body`` のサブツリー角運動量（ :ref:`subtreeangmom<sensor-subtreeangmom>` センサーで報告）であり、 :math:`\dot q` が一般化速度 ``mjData.qvel`` である場合、 :math:`h = H \dot q` です。

.. _mj_name2id:

`mj_name2id <#mj_name2id>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_name2id

指定された :ref:`mjtObj` タイプと名前を持つオブジェクトのIDを取得します。IDが見つからない場合は-1を返します。

.. _mj_id2name:

`mj_id2name <#mj_id2name>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_id2name

指定された :ref:`mjtObj` タイプとIDを持つオブジェクトの名前を取得します。名前が見つからない場合は ``NULL`` を返します。

.. _mj_fullM:

`mj_fullM <#mj_fullM>`__
~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_fullM

スパース慣性行列 ``M`` をフル（密）行列に変換します。
|br| ``dst`` のサイズは ``nv x nv`` 、 ``M`` は ``mjData.qM`` と同じ構造でなければなりません。

``mjData`` のメンバー ``qM`` と ``M`` は同じ行列を異なる形式で表現しています。前者はMuJoCo固有であり、後者は標準的なCompressed Sparse Row（下三角のみ）です。慣性行列の :math:`L^T D L` 分解 ``mjData.qLD`` は ``mjData.M`` と同じCSR形式を使用します。教育的な例については
`engine_support_test <https://github.com/google-deepmind/mujoco/blob/main/test/engine/engine_support_test.cc>`__ を参照してください。

.. _mj_mulM:

`mj_mulM <#mj_mulM>`__
~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_mulM

この関数は ``mjData.M`` に格納された関節空間慣性行列にベクトルを乗算します。

.. _mj_mulM2:

`mj_mulM2 <#mj_mulM2>`__
~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_mulM2

ベクトルに（慣性行列）^(1/2)を乗算します。

.. _mj_addM:

`mj_addM <#mj_addM>`__
~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_addM

慣性行列を宛先行列に加算します（下三角のみ）。

すべてのint*がNULLの場合、宛先はスパースまたは密行列が使用可能です。

*Nullable:* ``rownnz``, ``rowadr``, ``colind``

.. _mj_applyFT:

`mj_applyFT <#mj_applyFT>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_applyFT

この関数はボディ上の点にデカルト力とトルクを適用し、結果をすべての適用力のベクトル mjData.qfrc_applied に加算するために使用できます。この関数はこのベクトルへのポインタを必要とします。これは、結果を別のベクトルに加算したい場合があるためです。

.. _mj_objectVelocity:

`mj_objectVelocity <#mj_objectVelocity>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_objectVelocity

オブジェクト中心フレームでオブジェクトの6D速度（回転:並進）を計算します。ワールド/ローカル方向。

.. _mj_objectAcceleration:

`mj_objectAcceleration <#mj_objectAcceleration>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_objectAcceleration

オブジェクト中心フレームでオブジェクトの6D加速度（回転:並進）を計算します。ワールド/ローカル方向。モデルに加速度または力センサーが存在しない場合、制約ソルバーからの寄与を含む全ボディ加速度 mjData.cacc を計算するために :ref:`mj_rnePostConstraint` を手動で呼び出す必要があります。

.. _mj_geomDistance:

`mj_geomDistance <#mj_geomDistance>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_geomDistance

2つのジオム間の最小符号付き距離と、オプションで ``geom1`` から ``geom2`` へのセグメントを返します。
返される距離は ``distmax`` で上限が設定されます。 |br| ``distmax`` より小さい距離の衝突が見つからない場合、関数は ``distmax`` を返し、 ``fromto`` が指定されている場合は (0, 0, 0, 0, 0, 0) に設定されます。

*Nullable:* ``fromto``

.. admonition:: `nativeccd` での異なる（正しい）動作
   :class: note

   :ref:`衝突検出<coDistance>` で説明されているように、 :ref:`レガシーCCDパイプライン<coCCD>` を使用した場合、距離は不正確であり、その使用は推奨されない。

.. _mj_contactForce:

`mj_contactForce <#mj_contactForce>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_contactForce

接触IDが与えられた場合、接触フレームで6D力:トルクを抽出します。

.. _mj_differentiatePos:

`mj_differentiatePos <#mj_differentiatePos>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_differentiatePos

この関数はクォータニオンの性質を考慮しながら、qpos形式の2つのベクトルを減算します（そして結果をdtで割ります）。単位クォータニオンは空間的な方向を表すことを思い出してください。これらは4Dの単位球上の点です。その球の接線は回転速度の3D平面です。したがって、2つのクォータニオンを正しい方法で減算すると、結果は4Dベクトルではなく3Dベクトルになります。したがって、出力のqvelは次元nvを持ち、入力は次元nqを持ちます。

.. _mj_integratePos:

`mj_integratePos <#mj_integratePos>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_integratePos

これはmj_differentiatePosの逆の操作です。qvel形式のベクトル（dtでスケーリング）をqpos形式のベクトルに加算します。

.. _mj_normalizeQuat:

`mj_normalizeQuat <#mj_normalizeQuat>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_normalizeQuat

qpos形式のベクトル内のすべてのクォータニオンを正規化します。

.. _mj_local2Global:

`mj_local2Global <#mj_local2Global>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_local2Global

ボディローカル座標からグローバルデカルト座標へのマッピング。sameframeはmjtSameFrameから値を取ります。

.. _mj_getTotalmass:

`mj_getTotalmass <#mj_getTotalmass>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_getTotalmass

すべてのボディの質量を合計します。

.. _mj_setTotalmass:

`mj_setTotalmass <#mj_setTotalmass>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_setTotalmass

指定された全体質量を達成するようにボディの質量と慣性をスケーリングします。

.. _mj_getPluginConfig:

`mj_getPluginConfig <#mj_getPluginConfig>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_getPluginConfig

プラグインインスタンスの設定属性値を返します。

NULL: 無効なプラグインインスタンスIDまたは属性名

.. _mj_loadPluginLibrary:

`mj_loadPluginLibrary <#mj_loadPluginLibrary>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_loadPluginLibrary

動的ライブラリをロードする。この動的ライブラリは1つ以上のプラグインを登録するものと想定される。

.. _mj_loadAllPluginLibraries:

`mj_loadAllPluginLibraries <#mj_loadAllPluginLibraries>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_loadAllPluginLibraries

ディレクトリをスキャンし、すべての動的ライブラリをロードする。指定されたディレクトリ内の動的ライブラリは1つ以上のプラグインを登録するものと想定される。オプションで、コールバックが指定されている場合、プラグインを登録する各動的ライブラリに対してそのコールバックが呼び出される。

.. _mj_version:

`mj_version <#mj_version>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_version

バージョン番号を返します。1.0.2は102としてエンコードされます。

.. _mj_versionString:

`mj_versionString <#mj_versionString>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_versionString

MuJoCoの現在のバージョンをヌル終端文字列として返します。

.. _Components:

コンポーネント
^^^^^^^^^^^^^^^^^^^^^

これらはシミュレーションパイプラインのコンポーネントであり、 :ref:`mj_step` 、 :ref:`mj_forward` 、 :ref:`mj_inverse` から内部的に呼び出されます。ユーザーがこれらを呼び出す必要はほとんどありません。

.. _mj_fwdKinematics:

`mj_fwdKinematics <#mj_fwdKinematics>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_fwdKinematics

すべての運動学的計算（kinematics、comPos、camlight、flex、tendon）を実行します。

.. _mj_fwdPosition:

`mj_fwdPosition <#mj_fwdPosition>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_fwdPosition

位置依存の計算を実行します。

.. _mj_fwdVelocity:

`mj_fwdVelocity <#mj_fwdVelocity>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_fwdVelocity

速度依存の計算を実行します。

.. _mj_fwdActuation:

`mj_fwdActuation <#mj_fwdActuation>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_fwdActuation

アクチュエータ力 qfrc_actuator を計算します。

.. _mj_fwdAcceleration:

`mj_fwdAcceleration <#mj_fwdAcceleration>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_fwdAcceleration

すべての非制約力を合計し、qacc_smoothを計算します。

.. _mj_fwdConstraint:

`mj_fwdConstraint <#mj_fwdConstraint>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_fwdConstraint

選択された制約ソルバーを実行します。

.. _mj_Euler:

`mj_Euler <#mj_Euler>`__
~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_Euler

オイラー積分器、速度に対して半陰的。

.. _mj_RungeKutta:

`mj_RungeKutta <#mj_RungeKutta>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_RungeKutta

ルンゲ・クッタ陽的N次積分器。

.. _mj_implicit:

`mj_implicit <#mj_implicit>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_implicit

速度に対して陰的な積分器（"implicit"または"implicitfast"、 :ref:`数値積分<geIntegration>` を参照）を使用してシミュレーション状態を積分し、シミュレーション時間を進めます。この関数で計算されるフィールドについては `mjdata.h
<https://github.com/google-deepmind/mujoco/blob/main/include/mujoco/mjdata.h>`__ を参照してください。

.. _mj_invPosition:

`mj_invPosition <#mj_invPosition>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_invPosition

逆動力学の位置依存計算を実行します。

.. _mj_invVelocity:

`mj_invVelocity <#mj_invVelocity>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_invVelocity

逆動力学の速度依存計算を実行します。

.. _mj_invConstraint:

`mj_invConstraint <#mj_invConstraint>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_invConstraint

逆制約動力学の解析式を適用します。

.. _mj_compareFwdInv:

`mj_compareFwdInv <#mj_compareFwdInv>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_compareFwdInv

順動力学と逆動力学を比較し、結果をfwdinvに保存します。

.. _Subcomponents:

サブコンポーネント
^^^^^^^^^^^^^^^^^^^^^^^^^^^

これらはシミュレーションパイプラインのサブコンポーネントであり、上記のコンポーネントから内部的に呼び出されます。

.. _mj_sensorPos:

`mj_sensorPos <#mj_sensorPos>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_sensorPos

位置依存センサーを評価します。

.. _mj_sensorVel:

`mj_sensorVel <#mj_sensorVel>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_sensorVel

速度依存センサーを評価します。

.. _mj_sensorAcc:

`mj_sensorAcc <#mj_sensorAcc>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_sensorAcc

加速度および力依存センサーを評価します。

.. _mj_energyPos:

`mj_energyPos <#mj_energyPos>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_energyPos

位置依存エネルギー（ポテンシャル）を評価します。

.. _mj_energyVel:

`mj_energyVel <#mj_energyVel>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_energyVel

速度依存エネルギー（運動）を評価します。

.. _mj_checkPos:

`mj_checkPos <#mj_checkPos>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_checkPos

qposをチェックし、要素が大きすぎるかnanの場合はリセットします。

.. _mj_checkVel:

`mj_checkVel <#mj_checkVel>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_checkVel

qvelをチェックし、要素が大きすぎるかnanの場合はリセットする。

.. _mj_checkAcc:

`mj_checkAcc <#mj_checkAcc>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_checkAcc

qaccをチェックし、要素が大きすぎるかnanの場合はリセットする。

.. _mj_kinematics:

`mj_kinematics <#mj_kinematics>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_kinematics

順運動学を実行する。

.. _mj_comPos:

`mj_comPos <#mj_comPos>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_comPos

慣性と運動自由度を重心を中心としたグローバルフレームにマッピングする。

.. _mj_camlight:

`mj_camlight <#mj_camlight>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_camlight

カメラとライトの位置および向きを計算する。

.. _mj_flex:

`mj_flex <#mj_flex>`__
~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_flex

フレックス関連の量を計算する。

.. _mj_tendon:

`mj_tendon <#mj_tendon>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_tendon

テンドンの長さ、速度、モーメントアームを計算する。

.. _mj_transmission:

`mj_transmission <#mj_transmission>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_transmission

アクチュエータの伝達長さとモーメントを計算する。

.. _mj_crb:

`mj_crb <#mj_crb>`__
~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_crb

複合剛体慣性アルゴリズム（CRB）を実行する。

.. _mj_makeM:

`mj_makeM <#mj_makeM>`__
~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_makeM

:ref:`mj_crb` で複合剛体慣性を計算し、 :ref:`テンドンのアーマチュア<tendon-spatial-armature>` による項を加算する。関節空間慣性行列は ``mjData.qM`` と ``mjData.M`` の両方に格納される。これらの配列は同じ量を異なるレイアウト（親ベースと圧縮疎行形式）で表現している。

.. _mj_factorM:

`mj_factorM <#mj_factorM>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_factorM

慣性行列のスパース :math:`L^T D L` 分解を計算する。

.. _mj_solveM:

`mj_solveM <#mj_solveM>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_solveM

分解を使用して線形システム :math:`M x = y` を解く: :math:`x = (L^T D L)^{-1} y`

.. _mj_solveM2:

`mj_solveM2 <#mj_solveM2>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_solveM2

線形ソルブの半分: :math:`x = \sqrt{D^{-1}} (L^T)^{-1} y`

.. _mj_comVel:

`mj_comVel <#mj_comVel>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_comVel

cvel、cdof_dotを計算する。

.. _mj_passive:

`mj_passive <#mj_passive>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_passive

バネ・ダンパー、重力補償、流体力から qfrc_passive を計算する。

.. _mj_subtreeVel:

`mj_subtreeVel <#mj_subtreeVel>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_subtreeVel

サブツリーの線速度と角運動量: ``subtree_linvel`` 、 ``subtree_angmom`` を計算する。
この関数はモデルにサブツリーの :ref:`速度<sensor-subtreelinvel>` または
:ref:`運動量<sensor-subtreeangmom>` センサーが存在する場合に自動的にトリガーされる。
:ref:`ステージ<sensor-user-needstage>` が "vel" の :ref:`ユーザーセンサー<sensor-user>` に対してもトリガーされる。

.. _mj_rne:

`mj_rne <#mj_rne>`__
~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_rne

再帰的ニュートン・オイラー: :math:`M(q) \ddot q + C(q,\dot q)` を計算する。 ``flg_acc=0`` は慣性項を除去する（すなわち :math:`\ddot q = 0` と仮定）。

.. _mj_rnePostConstraint:

`mj_rnePostConstraint <#mj_rnePostConstraint>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_rnePostConstraint

最終計算された力と加速度を用いた再帰的ニュートン・オイラー。
ボディレベルの ``nv x 6`` 配列を3つ計算する。すべてsubtreecomベースの
:ref:`c-frame<tyNotesCom>` で定義され、 ``[rotation(3), translation(3)]`` の順に配置される。

- ``cacc``: ボディの加速度。 :ref:`mj_objectAcceleration` に必要。
- ``cfrc_int``: 親ボディとの相互作用力。
- ``cfrc_ext``: ボディに作用する外力。

この関数はモデルに以下のセンサーが存在する場合に自動的にトリガーされる:
:ref:`accelerometer<sensor-accelerometer>` 、 :ref:`force<sensor-force>` 、 :ref:`torque<sensor-torque>` 、
:ref:`framelinacc<sensor-framelinacc>` 、 :ref:`frameangacc<sensor-frameangacc>` 。
:ref:`ステージ<sensor-user-needstage>` が "acc" の :ref:`ユーザーセンサー<sensor-user>` に対してもトリガーされる。

計算された力の配列 ``cfrc_int`` と ``cfrc_ext`` には現在既知のバグがあり、空間テンドンの効果が考慮されていない。 :issue:`832` を参照。

.. _mj_collision:

`mj_collision <#mj_collision>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_collision

衝突検出を実行する。

.. _mj_makeConstraint:

`mj_makeConstraint <#mj_makeConstraint>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_makeConstraint

制約を構築する。

.. _mj_island:

`mj_island <#mj_island>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_island

制約アイランドを検出する。

.. _mj_projectConstraint:

`mj_projectConstraint <#mj_projectConstraint>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_projectConstraint

逆制約慣性 efc_AR を計算する。

.. _mj_referenceConstraint:

`mj_referenceConstraint <#mj_referenceConstraint>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_referenceConstraint

efc_vel、efc_arefを計算する。

.. _mj_constraintUpdate:

`mj_constraintUpdate <#mj_constraintUpdate>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_constraintUpdate

``efc_state`` 、 ``efc_force`` 、 ``qfrc_constraint`` 、および（オプションで）錐のヘッセ行列を計算する。
``cost`` が ``NULL`` でない場合、 ``*cost = s(jar)`` を設定する。ここで ``jar = Jac*qacc - aref`` 。

*Nullable:* ``cost``

.. _Raycollisions:

レイキャスティング
^^^^^^^^^^^^^^^^^^^^^^^^^^^

レイコリジョン（レイキャスティングとも呼ばれる）は、レイとジオムの交差距離 ``x`` を求める。レイは3D点 ``p`` から方向 ``v`` に射出される直線、すなわち ``(p + x*v, x >= 0)`` である。このファミリーのすべての関数は最も近いジオム表面までの距離を返すか、交差がない場合は -1 を返す。 ``p`` がジオムの内部にある場合、レイは内側から表面と交差するが、これも交差としてカウントされることに注意。

すべてのレイコリジョン関数は :ref:`mj_kinematics` （ :ref:`mjData` を参照）で計算される量に依存するため、 :ref:`mj_kinematics` またはそれを呼び出す関数（例: :ref:`mj_fwdPosition` ）の後に呼び出す必要がある。すべてのジオムタイプと交差するトップレベル関数は、単一レイをキャストする :ref:`mj_ray` と、単一点から複数レイをキャストする :ref:`mj_multiRay` である。

.. _mj_ray:

`mj_ray <#mj_ray>`__
~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_ray

レイ ``pnt+x*vec, x >= 0`` をジオムと交差させる。

- 最も近い表面までの距離 ``x`` を返すか、交差がない場合は -1 を返す。
- ``geomid`` が NULL でない場合、交差したジオムのIDを書き込む。交差がない場合は -1。
- ``normal`` が NULL でない場合、交差点での表面法線を書き込む。法線はレイの方向に関わらず常にジオムの **外側を向く** （つまり、内側から表面に当たるレイの場合も同様）。
- ID ``bodyexclude`` のボディ内のジオムを除外する。すべてのボディを含めるには -1 を使用する。
- ``geomgroup`` は長さ :ref:`mjNGROUP<glNumeric>` の配列で、1はそのグループを含めることを意味する。ジオムグループの除外をスキップするには NULL を渡す。
- ``flg_static`` が 0 の場合、静的ジオムは除外される。

*Nullable:* ``geomgroup``, ``geomid``, ``normal``

.. _mj_multiRay:

`mj_multiRay <#mj_multiRay>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_multiRay

単一点から射出される複数のレイを交差させ、指定された場合は法線を計算する。

mj_ray と同様のセマンティクスだが、vec、normal、distは配列である。

cutoff より遠いジオムは無視される。

*Nullable:* ``geomgroup``, ``geomid``, ``normal``

.. _mj_rayHfield:

`mj_rayHfield <#mj_rayHfield>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_rayHfield

レイをハイトフィールドと交差させる。最も近い距離を返すか、交差がない場合は -1 を返す。

*Nullable:* ``normal``

.. _mj_rayMesh:

`mj_rayMesh <#mj_rayMesh>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_rayMesh

レイをメッシュと交差させる。最も近い距離を返すか、交差がない場合は -1 を返す。

*Nullable:* ``normal``

.. _mju_rayGeom:

`mju_rayGeom <#mju_rayGeom>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_rayGeom

レイを純粋なジオムと交差させる。最も近い距離を返すか、交差がない場合は -1 を返す。

*Nullable:* ``normal``

.. _mj_rayFlex:

`mj_rayFlex <#mj_rayFlex>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_rayFlex

レイをフレックスと交差させる。最も近い距離を返すか、交差がない場合は -1 を返す。最も近い頂点IDと表面法線も出力する。

*Nullable:* ``vertid``, ``normal``

.. _mju_raySkin:

`mju_raySkin <#mju_raySkin>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_raySkin

レイをスキンと交差させる。最も近い距離を返すか、交差がない場合は -1 を返す。最も近い頂点IDも出力する。

*Nullable:* ``vertid``

.. _Printing:

出力
^^^^^^^^

これらの関数は、デバッグ目的で様々な量を画面に出力するために使用できます。

.. _mj_printFormattedModel:

`mj_printFormattedModel <#mj_printFormattedModel>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_printFormattedModel

フォーマットを指定して mjModel をテキストファイルに出力します。
float_formatは単一のfloat値に対する有効なprintf形式のフォーマット文字列でなければなりません。

.. _mj_printModel:

`mj_printModel <#mj_printModel>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_printModel

モデルをテキストファイルに出力します。

.. _mj_printFormattedData:

`mj_printFormattedData <#mj_printFormattedData>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_printFormattedData

フォーマットを指定して mjData をテキストファイルに出力します。
float_formatは単一のfloat値に対する有効なprintf形式のフォーマット文字列でなければなりません。

.. _mj_printData:

`mj_printData <#mj_printData>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_printData

データをテキストファイルに出力します。

.. _mju_printMat:

`mju_printMat <#mju_printMat>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_printMat

行列を画面に出力します。

.. _mju_printMatSparse:

`mju_printMatSparse <#mju_printMatSparse>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_printMatSparse

疎行列を画面に出力します。

.. _mj_printSchema:

`mj_printSchema <#mj_printSchema>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_printSchema

内部XMLスキーマをプレーンテキストまたはHTMLとして出力します。スタイルパディングまたは ``&nbsp;`` を使用します。

.. _mj_printScene:

`mj_printScene <#mj_printScene>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_printScene

シーンをテキストファイルに出力します。

.. _mj_printFormattedScene:

`mj_printFormattedScene <#mj_printFormattedScene>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_printFormattedScene

フォーマットを指定してシーンをテキストファイルに出力します。
float_formatは単一のfloat値に対する有効なprintf形式のフォーマット文字列でなければなりません。

.. _Virtualfilesystem:

仮想ファイルシステム
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

仮想ファイルシステム（VFS）は、MJBバイナリモデルファイル、XMLファイル（MJCF、URDFおよびインクルードファイル）、STLメッシュ、テクスチャおよびハイトフィールド用のPNG、カスタムハイトフィールド形式のHFファイルなど、必要なすべてのファイルをメモリにロードすることを可能にする。VFS内のモデルファイルとリソースファイルは、プログラムで構築することもできる（例えばメモリに書き込むPythonライブラリを使用して）。必要なすべてのファイルがVFSに格納されたら、ユーザーは :ref:`mj_loadModel` または :ref:`mj_loadXML` をVFSへのポインタ付きで呼び出すことができる。このポインタが NULL でない場合、ローダーはロードしようとするすべてのファイルについてまずVFSをチェックし、VFSにファイルが見つからない場合にのみディスクにアクセスする。

VFSはまず :ref:`mj_defaultVFS` で割り当て、 :ref:`mj_deleteVFS` で解放する必要がある。


.. _mj_defaultVFS:

`mj_defaultVFS <#mj_defaultVFS>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_defaultVFS

空のVFSを初期化する。VFSの割り当て解除には :ref:`mj_deleteVFS` を呼び出す必要がある。

.. _mj_mountVFS:

`mj_mountVFS <#mj_mountVFS>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_mountVFS

指定されたパスでファイル操作を処理する ResourceProvider をマウントする。戻り値: 0: 成功、2: 名前重複、-1: 無効なリソースプロバイダ。

.. _mj_unmountVFS:

`mj_unmountVFS <#mj_unmountVFS>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_unmountVFS

以前にマウントした ResourceProvider をアンマウントする。戻り値: 0: 成功、-1: VFSに見つからない。

.. _mj_addFileVFS:

`mj_addFileVFS <#mj_addFileVFS>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_addFileVFS

VFSにファイルを追加する。directory引数はオプションで、NULLまたは空にできる。成功時は0、名前衝突時は2、内部エラー発生時は-1を返す。

*Nullable:* ``directory``

.. _mj_addBufferVFS:

`mj_addBufferVFS <#mj_addBufferVFS>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_addBufferVFS

バッファからVFSにファイルを追加する。戻り値: 0: 成功、2: 名前重複、-1: ロード失敗。

.. _mj_deleteFileVFS:

`mj_deleteFileVFS <#mj_deleteFileVFS>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_deleteFileVFS

VFSからファイルを削除する。戻り値: 0: 成功、-1: VFSに見つからない。

.. _mj_deleteVFS:

`mj_deleteVFS <#mj_deleteVFS>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_deleteVFS

VFSからすべてのファイルを削除し、VFSの内部メモリを割り当て解除する。

.. _Assetcache:

アセットキャッシュ
^^^^^^^^^^^^^^^^^^^^^^^^^^^

アセットキャッシュは、繰り返しの遅い再コンパイルを避けるためにアセット（テクスチャ、メッシュなど）をキャッシュする仕組みである。以下のメソッドはキャッシュの容量を制御したり、完全に無効化する方法を提供する。

.. _mj_getCacheSize:

`mj_getCacheSize <#mj_getCacheSize>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_getCacheSize

アセットキャッシュの現在のサイズをバイト単位で取得する。

.. _mj_getCacheCapacity:

`mj_getCacheCapacity <#mj_getCacheCapacity>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_getCacheCapacity

アセットキャッシュの容量をバイト単位で取得する。

.. _mj_setCacheCapacity:

`mj_setCacheCapacity <#mj_setCacheCapacity>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_setCacheCapacity

アセットキャッシュの容量をバイト単位で設定する（0で無効化）。新しい容量を返す。

.. _mj_getCache:

`mj_getCache <#mj_getCache>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_getCache

コンパイラが使用する内部アセットキャッシュを取得する。

.. _mj_clearCache:

`mj_clearCache <#mj_clearCache>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_clearCache

アセットキャッシュをクリアする。

.. _Initialization:

初期化
^^^^^^^^^^^^^^

このセクションには、モデルやその他のデータ構造をロード/初期化する関数が含まれている。これらの使用方法はコードサンプルで詳しく説明されている。

.. _mj_defaultLROpt:

`mj_defaultLROpt <#mj_defaultLROpt>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_defaultLROpt

長さ範囲計算のデフォルトオプションを設定する。

.. _mj_defaultSolRefImp:

`mj_defaultSolRefImp <#mj_defaultSolRefImp>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_defaultSolRefImp

ソルバーパラメータをデフォルト値に設定する。

*Nullable:* ``solref``, ``solimp``

.. _mj_defaultOption:

`mj_defaultOption <#mj_defaultOption>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_defaultOption

物理オプションをデフォルト値に設定する。

.. _mj_defaultVisual:

`mj_defaultVisual <#mj_defaultVisual>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_defaultVisual

可視化オプションをデフォルト値に設定する。

.. _mj_copyModel:

`mj_copyModel <#mj_copyModel>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_copyModel

mjModelをコピーする。destが NULL の場合は新しく割り当てる。

*Nullable:* ``dest``

.. _mj_saveModel:

`mj_saveModel <#mj_saveModel>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_saveModel

モデルをバイナリMJBファイルまたはメモリバッファに保存する。バッファが指定された場合はバッファが優先される。

*Nullable:* ``filename``, ``buffer``

.. _mj_loadModel:

`mj_loadModel <#mj_loadModel>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_loadModel

バイナリMJBファイルからモデルをロードする。

vfsが NULL でない場合、ディスクから読み込む前にvfs内のファイルを検索する。

*Nullable:* ``vfs``

.. _mj_loadModelBuffer:

`mj_loadModelBuffer <#mj_loadModelBuffer>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_loadModelBuffer

メモリバッファからモデルをロードする。

.. _mj_deleteModel:

`mj_deleteModel <#mj_deleteModel>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_deleteModel

モデルのメモリ割り当てを解放する。

.. _mj_sizeModel:

`mj_sizeModel <#mj_sizeModel>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_sizeModel

モデルを保持するために必要なバッファサイズを返す。

.. _mj_makeData:

`mj_makeData <#mj_makeData>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_makeData

指定されたモデルに対応する mjData を割り当てる。

モデルバッファが未割り当ての場合、初期構成は設定されない。

.. _mj_copyData:

`mj_copyData <#mj_copyData>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_copyData

mjDataをコピーする。
mは MJMODEL_INTS のサイズフィールドのみを含む必要がある。

.. _mjv_copyData:

`mjv_copyData <#mjv_copyData>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjv_copyData

mjDataをコピーする。可視化に不要な大きな配列はスキップする。

.. _mj_resetData:

`mj_resetData <#mj_resetData>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_resetData

データをデフォルトにリセットする。

.. _mj_resetDataDebug:

`mj_resetDataDebug <#mj_resetDataDebug>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_resetDataDebug

データをデフォルトにリセットし、それ以外をすべて debug_value で埋める。

.. _mj_resetDataKeyframe:

`mj_resetDataKeyframe <#mj_resetDataKeyframe>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_resetDataKeyframe

データをリセットする。0 <= key < nkey の場合、指定されたキーフレームからフィールドを設定する。

.. _mj_markStack:

`mj_markStack <#mj_markStack>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_markStack

mjDataスタックに新しいフレームをマークする。

.. _mj_freeStack:

`mj_freeStack <#mj_freeStack>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_freeStack

現在の mjData スタックフレームを解放する。最後の mj_markStack の呼び出し以降に mj_stackAlloc が返したすべてのポインタは、以降使用してはならない。

.. _mj_stackAllocByte:

`mj_stackAllocByte <#mj_stackAllocByte>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_stackAllocByte

指定されたアラインメントで mjData スタック上にバイト数を割り当てる。

スタックオーバーフロー時に mju_error を呼び出す。

.. _mj_stackAllocNum:

`mj_stackAllocNum <#mj_stackAllocNum>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_stackAllocNum

mjDataスタック上にmjtNum配列を割り当てる。スタックオーバーフロー時にmju_errorを呼び出す。

.. _mj_stackAllocInt:

`mj_stackAllocInt <#mj_stackAllocInt>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_stackAllocInt

mjDataスタック上にint配列を割り当てる。スタックオーバーフロー時にmju_errorを呼び出す。

.. _mj_deleteData:

`mj_deleteData <#mj_deleteData>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_deleteData

mjDataのメモリ割り当てを解放する。

.. _mj_resetCallbacks:

`mj_resetCallbacks <#mj_resetCallbacks>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_resetCallbacks

すべてのコールバックを NULL ポインタにリセットする（NULL がデフォルト）。

.. _mj_setConst:

`mj_setConst <#mj_setConst>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_setConst

qpos0構成に対応する mjModel の定数フィールドを設定する。

.. _mj_setLengthRange:

`mj_setLengthRange <#mj_setLengthRange>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_setLengthRange

指定されたアクチュエータの actuator_lengthrange を設定する。成功時は1、エラー時は0を返す。

*Nullable:* ``error``

.. _mj_makeSpec:

`mj_makeSpec <#mj_makeSpec>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_makeSpec

空のspecを作成する。

.. _mj_copySpec:

`mj_copySpec <#mj_copySpec>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_copySpec

specをコピーする。

.. _mj_deleteSpec:

`mj_deleteSpec <#mj_deleteSpec>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_deleteSpec

mjSpecのメモリ割り当てを解放する。

.. _mjs_activatePlugin:

`mjs_activatePlugin <#mjs_activatePlugin>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_activatePlugin

プラグインを有効化する。成功時は0を返す。

.. _mjs_setDeepCopy:

`mjs_setDeepCopy <#mjs_setDeepCopy>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_setDeepCopy

アタッチ時のディープコピーのオン/オフを切り替える。成功時は0を返す。

.. _Errorandmemory:

エラーとメモリ
^^^^^^^^^^^^^^^^^^^^^

.. _mju_error:

`mju_error <#mju_error>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_error

メインエラー関数。呼び出し元に戻らない。

.. _mju_error_i:

`mju_error_i <#mju_error_i>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_error_i

非推奨: mju_error を使用すること。

.. _mju_error_s:

`mju_error_s <#mju_error_s>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_error_s

非推奨: mju_error を使用すること。

.. _mju_warning:

`mju_warning <#mju_warning>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_warning

メイン警告関数。呼び出し元に戻る。

.. _mju_warning_i:

`mju_warning_i <#mju_warning_i>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_warning_i

非推奨: mju_warning を使用すること。

.. _mju_warning_s:

`mju_warning_s <#mju_warning_s>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_warning_s

非推奨: mju_warning を使用すること。

.. _mju_clearHandlers:

`mju_clearHandlers <#mju_clearHandlers>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_clearHandlers

ユーザーエラーおよびメモリハンドラをクリアする。

.. _mju_malloc:

`mju_malloc <#mju_malloc>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_malloc

メモリを割り当てる。64バイトアラインメント、サイズを64の倍数にパディングする。

.. _mju_free:

`mju_free <#mju_free>`__
~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_free

メモリを解放する。デフォルトでは free() を使用する。

.. _mj_warning:

`mj_warning <#mj_warning>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mj_warning

高レベル警告関数: mjData 内の警告をカウントし、最初の警告のみ出力する。

.. _mju_writeLog:

`mju_writeLog <#mju_writeLog>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_writeLog

[datetime, type: message] を MUJOCO_LOG.TXT に書き込む。

.. _mjs_getError:

`mjs_getError <#mjs_getError>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_getError

spec からコンパイラエラーメッセージを取得する。

.. _mjs_isWarning:

`mjs_isWarning <#mjs_isWarning>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_isWarning

コンパイラエラーが警告の場合に1を返す。

.. _Miscellaneous:

その他
^^^^^^^^^^^^^

.. _mju_muscleGain:

`mju_muscleGain <#mju_muscleGain>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_muscleGain

筋肉の能動力、prm = (range[2], force, scale, lmin, lmax, vmax, fpmax, fvmax)。

.. _mju_muscleBias:

`mju_muscleBias <#mju_muscleBias>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_muscleBias

筋肉の受動力、prm = (range[2], force, scale, lmin, lmax, vmax, fpmax, fvmax)。

.. _mju_muscleDynamics:

`mju_muscleDynamics <#mju_muscleDynamics>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_muscleDynamics

筋肉の活性化ダイナミクス、prm = (tau_act, tau_deact, smoothing_width)。

.. _mju_encodePyramid:

`mju_encodePyramid <#mju_encodePyramid>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_encodePyramid

接触力を角錐表現に変換する。

.. _mju_decodePyramid:

`mju_decodePyramid <#mju_decodePyramid>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_decodePyramid

角錐表現を接触力に変換する。

.. _mju_springDamper:

`mju_springDamper <#mju_springDamper>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_springDamper

バネ・ダンパーを解析的に積分する。pos(dt) を返す。

.. _mju_min:

`mju_min <#mju_min>`__
~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_min

a と b を1回ずつ評価して min(a,b) を返す。

.. _mju_max:

`mju_max <#mju_max>`__
~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_max

a と b を1回ずつ評価して max(a,b) を返す。

.. _mju_clip:

`mju_clip <#mju_clip>`__
~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_clip

x を範囲 [min, max] にクリップする。

.. _mju_sign:

`mju_sign <#mju_sign>`__
~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_sign

x の符号を返す: +1、-1 または 0。

.. _mju_round:

`mju_round <#mju_round>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_round

x を最も近い整数に丸める。

.. _mju_type2Str:

`mju_type2Str <#mju_type2Str>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_type2Str

型ID（mjtObj）を型名に変換する。

.. _mju_str2Type:

`mju_str2Type <#mju_str2Type>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_str2Type

型名を型ID（mjtObj）に変換する。

.. _mju_writeNumBytes:

`mju_writeNumBytes <#mju_writeNumBytes>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_writeNumBytes

標準的な文字サフィックスを使用して、人間が読める形式のバイト数を返す。

.. _mju_warningText:

`mju_warningText <#mju_warningText>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_warningText

警告タイプと情報から警告メッセージを構築する。

.. _mju_isBad:

`mju_isBad <#mju_isBad>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_isBad

nan または abs(x)>mjMAXVAL の場合は1、それ以外は0を返す。チェック関数で使用される。

.. _mju_isZero:

`mju_isZero <#mju_isZero>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_isZero

すべての要素が0の場合は1を返す。

.. _mju_standardNormal:

`mju_standardNormal <#mju_standardNormal>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_standardNormal

標準正規乱数生成器（オプションで2番目の数値も生成）。

.. _mju_f2n:

`mju_f2n <#mju_f2n>`__
~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_f2n

float から mjtNum に変換する。

.. _mju_n2f:

`mju_n2f <#mju_n2f>`__
~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_n2f

mjtNum から float に変換する。

.. _mju_d2n:

`mju_d2n <#mju_d2n>`__
~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_d2n

double から mjtNum に変換する。

.. _mju_n2d:

`mju_n2d <#mju_n2d>`__
~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_n2d

mjtNum から double に変換する。

.. _mju_insertionSort:

`mju_insertionSort <#mju_insertionSort>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_insertionSort

挿入ソート。結果のリストは昇順になる。

.. _mju_insertionSortInt:

`mju_insertionSortInt <#mju_insertionSortInt>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_insertionSortInt

整数の挿入ソート。結果のリストは昇順になる。

.. _mju_Halton:

`mju_Halton <#mju_Halton>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_Halton

ハルトン列を生成する。

.. _mju_strncpy:

`mju_strncpy <#mju_strncpy>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_strncpy

strncpy を呼び出し、dst[n-1] = 0 を設定する。

.. _mju_sigmoid:

`mju_sigmoid <#mju_sigmoid>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_sigmoid

5次多項式を使用した2回連続微分可能なシグモイド関数:

.. math::
   s(x) =
   \begin{cases}
      0,                    &       & x \le 0  \\
      6x^5 - 15x^4 + 10x^3, & 0 \lt & x \lt 1  \\
      1,                    & 1 \le & x \qquad
   \end{cases}

.. _Interaction:

インタラクション
^^^^^^^^^^^^^^^^^^^^^^^^

これらの関数は抽象的なマウスインタラクションを実装し、カメラと摂動の制御を可能にする。これらの使用方法は :ref:`simulate<saSimulate>` で詳しく説明されている。

.. _mjv_defaultCamera:

`mjv_defaultCamera <#mjv_defaultCamera>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjv_defaultCamera

デフォルトカメラを設定する。

.. _mjv_defaultFreeCamera:

`mjv_defaultFreeCamera <#mjv_defaultFreeCamera>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjv_defaultFreeCamera

デフォルトのフリーカメラを設定する。

.. _mjv_defaultPerturb:

`mjv_defaultPerturb <#mjv_defaultPerturb>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjv_defaultPerturb

デフォルトの摂動を設定する。

.. _mjv_room2model:

`mjv_room2model <#mjv_room2model>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjv_room2model

姿勢をルーム空間からモデル空間に変換する。

.. _mjv_model2room:

`mjv_model2room <#mjv_model2room>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjv_model2room

姿勢をモデル空間からルーム空間に変換する。

.. _mjv_cameraInModel:

`mjv_cameraInModel <#mjv_cameraInModel>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjv_cameraInModel

モデル空間でカメラ情報を取得する。左右のOpenGLカメラを平均化する。

.. _mjv_cameraInRoom:

`mjv_cameraInRoom <#mjv_cameraInRoom>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjv_cameraInRoom

ルーム空間でカメラ情報を取得する。左右のOpenGLカメラを平均化する。

.. _mjv_frustumHeight:

`mjv_frustumHeight <#mjv_frustumHeight>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjv_frustumHeight

カメラから単位距離での錐台の高さを取得する。左右のOpenGLカメラを平均化する。

.. _mjv_alignToCamera:

`mjv_alignToCamera <#mjv_alignToCamera>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjv_alignToCamera

3Dベクトルを水平面で (0,1) と (forward_x,forward_y) の間の角度だけ回転させる。

.. _mjv_moveCamera:

`mjv_moveCamera <#mjv_moveCamera>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjv_moveCamera

マウスでカメラを移動する。action は mjtMouse。

.. _mjv_movePerturb:

`mjv_movePerturb <#mjv_movePerturb>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjv_movePerturb

マウスで摂動オブジェクトを移動する。action は mjtMouse。

.. _mjv_moveModel:

`mjv_moveModel <#mjv_moveModel>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjv_moveModel

マウスでモデルを移動する。action は mjtMouse。

.. _mjv_initPerturb:

`mjv_initPerturb <#mjv_initPerturb>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjv_initPerturb

選択されたボディから摂動の pos,quat をコピーする。摂動のスケールを設定する。

.. _mjv_applyPerturbPose:

`mjv_applyPerturbPose <#mjv_applyPerturbPose>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjv_applyPerturbPose

選択されたボディが mocap の場合は d->mocap に、それ以外は d->qpos に摂動の pos,quat を設定する。

flg_paused かつ選択されたボディのサブツリーのルートにフリージョイントがある場合のみ d->qpos を書き込む。

.. _mjv_applyPerturbForce:

`mjv_applyPerturbForce <#mjv_applyPerturbForce>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjv_applyPerturbForce

選択されたボディが動的な場合、d->xfrc_applied に摂動の force,torque を設定する。

.. _mjv_averageCamera:

`mjv_averageCamera <#mjv_averageCamera>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjv_averageCamera

2つのOpenGLカメラの平均を返す。

.. _mjv_select:

`mjv_select <#mjv_select>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjv_select

この関数はレイの交差を利用したマウス選択に使用される。aspectratio はビューポートの幅/高さ。relx と rely はビューポート内の対象2D点（通常はマウスカーソル）の相対座標である。この関数は指定された2D点の下にあるジオムのIDを返すか、ジオムがない場合は -1 を返す（スカイボックスが存在してもモデルジオムではないことに注意）。クリックされた点の3D座標は selpnt に返される。例は :ref:`simulate<saSimulate>` を参照。

.. _Visualization-api:

可視化
^^^^^^^^^^^^^

このセクションの関数は抽象的な可視化を実装します。その結果はOpenGLレンダラーによって使用され、独自のレンダラーを実装したいユーザーや、MuJoCoをUnityやUnreal Engineなどの高度なレンダリングツールに接続したいユーザーも使用できます。これらの関数の使用方法については :ref:`simulate<saSimulate>` を参照してください。

.. _mjv_defaultOption:

`mjv_defaultOption <#mjv_defaultOption>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjv_defaultOption

デフォルトの可視化オプションを設定します。

.. _mjv_defaultFigure:

`mjv_defaultFigure <#mjv_defaultFigure>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjv_defaultFigure

デフォルトのフィギュアを設定します。

.. _mjv_initGeom:

`mjv_initGeom <#mjv_initGeom>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjv_initGeom

NULLでない場合は指定されたジオムフィールドを初期化し、残りをデフォルト値に設定します。

*Nullable:* ``size``, ``pos``, ``mat``, ``rgba``

.. _mjv_connector:

`mjv_connector <#mjv_connector>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjv_connector

指定された2点間のコネクタタイプジオムの (type, size, pos, mat) を設定します。

他のすべてのプロパティを設定するために mjv_initGeom が既に呼び出されていることを前提とします。

mjGEOM_LINE の幅はピクセル単位です。

.. _mjv_defaultScene:

`mjv_defaultScene <#mjv_defaultScene>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjv_defaultScene

デフォルトの抽象シーンを設定します。

.. _mjv_makeScene:

`mjv_makeScene <#mjv_makeScene>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjv_makeScene

抽象シーンにリソースを割り当てます。

.. _mjv_freeScene:

`mjv_freeScene <#mjv_freeScene>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjv_freeScene

抽象シーンを解放します。

.. _mjv_updateScene:

`mjv_updateScene <#mjv_updateScene>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjv_updateScene

モデルの状態に基づいてシーン全体を更新します。

.. _mjv_copyModel:

`mjv_copyModel <#mjv_copyModel>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjv_copyModel

mjModelをコピーします。抽象的な可視化に不要な大きな配列はスキップします。

*Nullable:* ``dest``

.. _mjv_addGeoms:

`mjv_addGeoms <#mjv_addGeoms>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjv_addGeoms

選択されたカテゴリからジオムを追加します。

.. _mjv_makeLights:

`mjv_makeLights <#mjv_makeLights>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjv_makeLights

ライトのリストを作成します。

.. _mjv_updateCamera:

`mjv_updateCamera <#mjv_updateCamera>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjv_updateCamera

カメラを更新します。

.. _mjv_updateSkin:

`mjv_updateSkin <#mjv_updateSkin>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjv_updateSkin

スキンを更新します。

.. _mjv_cameraFrame:

`mjv_cameraFrame <#mjv_cameraFrame>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjv_cameraFrame

カメラの位置と前方、上方、右方ベクトルを計算します。

*Nullable:* ``headpos``, ``forward``, ``up``, ``right``

.. _mjv_cameraFrustum:

`mjv_cameraFrustum <#mjv_cameraFrustum>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjv_cameraFrustum

カメラの錐台を計算します：垂直、水平、およびクリップ平面。

*Nullable:* ``zver``, ``zhor``, ``zclip``

.. _OpenGLrendering:

OpenGLレンダリング
^^^^^^^^^^^^^^^^^^^^^^^^

これらの関数はOpenGLレンダラーを公開します。これらの関数の使用方法については :ref:`simulate<saSimulate>` を参照してください。

.. _mjr_defaultContext:

`mjr_defaultContext <#mjr_defaultContext>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjr_defaultContext

デフォルトの mjrContext を設定します。

.. _mjr_makeContext:

`mjr_makeContext <#mjr_makeContext>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjr_makeContext

カスタムOpenGLコンテキストにリソースを割り当てます。fontscaleは mjtFontScale です。

.. _mjr_changeFont:

`mjr_changeFont <#mjr_changeFont>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjr_changeFont

既存のコンテキストのフォントを変更します。

.. _mjr_addAux:

`mjr_addAux <#mjr_addAux>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjr_addAux

指定されたインデックスのAuxバッファをコンテキストに追加します。以前のAuxバッファは解放されます。

.. _mjr_freeContext:

`mjr_freeContext <#mjr_freeContext>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjr_freeContext

カスタムOpenGLコンテキストのリソースを解放し、デフォルトに設定します。

.. _mjr_resizeOffscreen:

`mjr_resizeOffscreen <#mjr_resizeOffscreen>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjr_resizeOffscreen

オフスクリーンバッファのサイズを変更します。

.. _mjr_uploadTexture:

`mjr_uploadTexture <#mjr_uploadTexture>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjr_uploadTexture

テクスチャをGPUにアップロードします。以前のアップロードがあれば上書きします。

.. _mjr_uploadMesh:

`mjr_uploadMesh <#mjr_uploadMesh>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjr_uploadMesh

メッシュをGPUにアップロードします。以前のアップロードがあれば上書きします。

.. _mjr_uploadHField:

`mjr_uploadHField <#mjr_uploadHField>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjr_uploadHField

ハイトフィールドをGPUにアップロードします。以前のアップロードがあれば上書きします。

.. _mjr_restoreBuffer:

`mjr_restoreBuffer <#mjr_restoreBuffer>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjr_restoreBuffer

con->currentBuffer を再びカレントにします。

.. _mjr_setBuffer:

`mjr_setBuffer <#mjr_setBuffer>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjr_setBuffer

レンダリング用のOpenGLフレームバッファを設定します：mjFB_WINDOW または mjFB_OFFSCREEN。

利用可能なバッファが1つだけの場合、そのバッファを設定し、framebuffer引数を無視します。

.. _mjr_readPixels:

`mjr_readPixels <#mjr_readPixels>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjr_readPixels

現在のOpenGLフレームバッファからクライアントバッファにピクセルを読み取ります。

ビューポートはOpenGLフレームバッファ内にあります。クライアントバッファは (0,0) から始まります。

.. _mjr_drawPixels:

`mjr_drawPixels <#mjr_drawPixels>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjr_drawPixels

クライアントバッファから現在のOpenGLフレームバッファにピクセルを描画します。

ビューポートはOpenGLフレームバッファ内にあります。クライアントバッファは (0,0) から始まります。

.. _mjr_blitBuffer:

`mjr_blitBuffer <#mjr_blitBuffer>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjr_blitBuffer

現在のフレームバッファのsrcビューポートから別のフレームバッファのdstビューポートにブリットします。

src、dstのサイズが異なり flg_depth==0 の場合、色は GL_LINEAR で補間されます。

.. _mjr_setAux:

`mjr_setAux <#mjr_setAux>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjr_setAux

カスタムOpenGLレンダリング用のAuxバッファを設定します（完了時に restoreBuffer を呼び出してください）。

.. _mjr_blitAux:

`mjr_blitAux <#mjr_blitAux>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjr_blitAux

Auxバッファから con->currentBuffer にブリットします。

.. _mjr_text:

`mjr_text <#mjr_text>`__
~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjr_text

相対座標 (x,y) にテキストを描画します。fontは mjtFont です。

.. _mjr_overlay:

`mjr_overlay <#mjr_overlay>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjr_overlay

テキストオーバーレイを描画します。fontは mjtFont、gridposは mjtGridPos です。

.. _mjr_maxViewport:

`mjr_maxViewport <#mjr_maxViewport>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjr_maxViewport

アクティブバッファの最大ビューポートを取得します。

.. _mjr_rectangle:

`mjr_rectangle <#mjr_rectangle>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjr_rectangle

矩形を描画します。

.. _mjr_label:

`mjr_label <#mjr_label>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjr_label

中央揃えテキスト付きの矩形を描画します。

.. _mjr_figure:

`mjr_figure <#mjr_figure>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjr_figure

2Dフィギュアを描画します。

.. _mjr_render:

`mjr_render <#mjr_render>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjr_render

3Dシーンをレンダリングします。

.. _mjr_finish:

`mjr_finish <#mjr_finish>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjr_finish

glFinish を呼び出します。

.. _mjr_getError:

`mjr_getError <#mjr_getError>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjr_getError

glGetError を呼び出し、結果を返します。

.. _mjr_findRect:

`mjr_findRect <#mjr_findRect>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjr_findRect

マウスを含む最初の矩形を検索します。見つからない場合は -1 を返します。

.. _UIframework:

UIフレームワーク
^^^^^^^^^^^^^^^^^^^^^^^

UIフレームワークの概要については :ref:`UI` を参照してください。

.. _mjui_themeSpacing:

`mjui_themeSpacing <#mjui_themeSpacing>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjui_themeSpacing

組み込みUIテーマのスペーシングを取得します（ind: 0-1）。

.. _mjui_themeColor:

`mjui_themeColor <#mjui_themeColor>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjui_themeColor

組み込みUIテーマの色を取得します（ind: 0-3）。

.. _mjui_add:

`mjui_add <#mjui_add>`__
~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjui_add

UIを構築するために使用されるヘルパー関数です。第2引数は :ref:`mjuiDef` 構造体の配列を指し、各要素が1つのアイテムに対応します。最後の（未使用の）アイテムはtypeが -1 に設定され、終端を示します。アイテムは最後に使用されたセクションの末尾の後に追加されます。この関数には別のバージョン (:ref:`mjui_addToSection<mjui_addToSection>`) もあり、UIの末尾ではなく指定されたセクションにアイテムを追加します。セクション数とセクションあたりのアイテム数には事前に割り当てられた最大値があり、それぞれ :ref:`mjMAXUISECT<glNumeric>` と :ref:`mjMAXUIITEM<glNumeric>` で指定されることに注意してください。これらの最大値を超えると低レベルエラーが発生します。

.. _mjui_addToSection:

`mjui_addToSection <#mjui_addToSection>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjui_addToSection

UIセクションに定義を追加します。

.. _mjui_resize:

`mjui_resize <#mjui_resize>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjui_resize

UIのサイズを計算します。

.. _mjui_update:

`mjui_update <#mjui_update>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjui_update

メインのUI更新関数です。ユーザーデータ（アイテムのデータポインタが指すもの）が変更されたとき、またはUI状態自体が変更されたときに呼び出す必要があります。通常、ユーザーが実装した上位レベルの関数（ :ref:`simulate.cc <saSimulate>` の ``UiModify`` ）から呼び出され、そこですべての矩形と関連する補助バッファのレイアウトも再計算されます。この関数はオフスクリーンOpenGLバッファのピクセルを更新します。最小限の更新を行うために、ユーザーは変更されたセクションとアイテムを指定します。-1 の値は、すべてのアイテムおよび/またはセクションを更新する必要があることを意味します（大きな変更の後に必要です）。

.. _mjui_event:

`mjui_event <#mjui_event>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjui_event

この関数は低レベルのイベントハンドラである。UIに必要な変更を加え、イベントを受け取ったアイテムへのポインタを返す（有効なイベントが記録されなかった場合は ``NULL`` ）。通常、ユーザーが実装したイベントハンドラ（ :ref:`simulate.cc <saSimulate>` の ``UiEvent`` ）内で呼び出され、どのUIアイテムが変更されたか、およびイベント処理後のそのアイテムの状態に応じてユーザーコードがアクションを実行する。

.. _mjui_render:

`mjui_render <#mjui_render>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjui_render

この関数は画面リフレッシュループで呼び出される。オフスクリーンOpenGLバッファをウィンドウフレームバッファにコピーする。アプリケーションに複数のUIがある場合、各UIに対して1回ずつ呼び出す必要がある。したがって ``mjui_render`` は常に呼び出されるが、 :ref:`mjui_update` はUIに変更があった場合のみ呼び出される。

.. _Derivatives-api:

導関数
^^^^^^^^^^^

以下の関数は、解析的および有限差分の両方の手法で、様々な関数の有用な導関数を提供する。後者は ``FD`` サフィックスの名前を持つ。APIの多くとは異なり、導関数関数の出力は先頭ではなく末尾の引数であることに注意。

.. _mjd_transitionFD:

`mjd_transitionFD <#mjd_transitionFD>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjd_transitionFD

有限差分による離散時間遷移行列を計算する。

mjData インスタンスにおける現在の :ref:`状態<siPhysicsState>` と :ref:`制御<siInput>` ベクトルを :math:`x, u` 、次の状態とセンサー値を :math:`y, s` とすると、トップレベルの :ref:`mj_step` 関数は :math:`(x,u) \rightarrow (y,s)` を計算する。 :ref:`mjd_transitionFD` は有限差分を使用して関連する4つのヤコビアンを計算する。これらの行列とその次元は:

.. csv-table::
   :header: "matrix", "Jacobian", "dimension"
   :widths: auto
   :align: left

   ``A``, :math:`\partial y / \partial x`, ``2*nv+na x 2*nv+na``
   ``B``, :math:`\partial y / \partial u`, ``2*nv+na x nu``
   ``C``, :math:`\partial s / \partial x`, ``nsensordata x 2*nv+na``
   ``D``, :math:`\partial s / \partial u`, ``nsensordata x nu``

- すべての出力はオプション（NULL可）。
- ``eps`` は有限差分のイプシロン。
- ``flg_centered`` は前進差分（0）または中心差分（1）のどちらを使用するかを示す。
- ルンゲ・クッタ積分器（ :ref:`mjINT_RK4<mjtIntegrator>` ）はサポートされない。

.. admonition:: 速度と精度の改善
   :class: tip

   ウォームスタート
     ウォームスタートが :ref:`無効<option-flag-warmstart>` でない場合、呼び出し時に存在するウォームスタート加速度
     ``mjData.qacc_warmstart`` は、決定論性を保持するためにすべての関連するパイプライン呼び出しの開始時にロードされる。ソルバー計算がシミュレーションのコストの大きい部分である場合、以下のトリックが大幅な高速化につながる可能性がある: まず :ref:`mj_forward` を呼び出してソルバーを収束させ、次に :ref:`ソルバーの反復回数<option-iterations>` を大幅に削減し、 :ref:`mjd_transitionFD` を呼び出し、最後に :ref:`iterations<option-iterations>` の元の値を復元する。すでに解の近くにいるため、新しい最小値を見つけるのに必要な反復回数は少ない。これは特に :ref:`Newton<option-solver>` ソルバーに当てはまり、最小値近傍での収束に必要な反復回数は1回程度になる可能性がある。

   トレランス
      ソルバーの :ref:`トレランス<option-tolerance>` を0に設定すると精度が向上する。これにより、ソルバーへのすべての呼び出しが正確に同じ反復回数を実行し、早期終了による数値誤差を防ぐことができる。もちろん、これは :ref:`ソルバーの反復回数<option-iterations>` を小さくすべきであることを意味する（最小値で無駄な計算をしないため）。この方法と上記の方法は組み合わせることができ、組み合わせるべきである。

*Nullable:* ``A``, ``B``, ``D``, ``C``

.. _mjd_inverseFD:

`mjd_inverseFD <#mjd_inverseFD>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjd_inverseFD

有限差分による連続時間逆動力学ヤコビアン。

mjData インスタンスにおける現在の :ref:`状態<siPhysicsState>` と加速度ベクトルを :math:`x, a` 、逆動力学で計算される力（ ``qfrc_inverse`` ）とセンサー値を :math:`f, s` とすると、関数 :ref:`mj_inverse` は :math:`(x,a) \rightarrow (f,s)` を計算する。 :ref:`mjd_inverseFD` は有限差分を使用して関連する7つのヤコビアンを計算する。これらの行列とその次元は:

.. csv-table::
   :header: "matrix", "Jacobian", "dimension"
   :widths: auto
   :align: left

   ``DfDq``, :math:`\partial f / \partial q`, ``nv x nv``
   ``DfDv``, :math:`\partial f / \partial v`, ``nv x nv``
   ``DfDa``, :math:`\partial f / \partial a`, ``nv x nv``
   ``DsDq``, :math:`\partial s / \partial q`, ``nv x nsensordata``
   ``DsDv``, :math:`\partial s / \partial v`, ``nv x nsensordata``
   ``DsDa``, :math:`\partial s / \partial a`, ``nv x nsensordata``
   ``DmDq``, :math:`\partial M / \partial q`, ``nv x nM``

- すべての出力はオプション（NULL可）。
- すべての出力は制御理論の慣例に対して転置されている（すなわち列優先）。
- ``DmDq`` は ``nv x nv x nv`` テンソル :math:`\partial M / \partial q` のスパース表現を含み、厳密には逆動力学ヤコビアンではないが、関連するアプリケーションで有用である。他の2つの :math:`\partial / \partial q` ヤコビアンのいずれかが要求された場合に必要な値がすでに計算されているため、ユーザーの利便性として提供される。
- ``eps`` は（前進）有限差分のイプシロン。
- ``flg_actuation`` は逆動力学の出力からアクチュエータ力（ ``qfrc_actuator`` ）を差し引くかどうかを示す。このフラグが正の場合、アクチュエータ力は外力として扱われない。
- モデルオプションフラグ ``invdiscrete`` は、正しい導関数情報を計算するために ``mjData.qacc`` の表現と対応している必要がある。

.. attention::
   - ルンゲ・クッタ4次積分器（ ``mjINT_RK4`` ）はサポートされない。
   - noslipソルバーはサポートされない。

*Nullable:* ``DfDq``, ``DfDv``, ``DfDa``, ``DsDq``, ``DsDv``, ``DsDa``, ``DmDq``

.. _mjd_subQuat:

`mjd_subQuat <#mjd_subQuat>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjd_subQuat

:ref:`mju_subQuat` （クォータニオン差分）の導関数。

*Nullable:* ``Da``, ``Db``

.. _mjd_quatIntegrate:

`mjd_quatIntegrate <#mjd_quatIntegrate>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjd_quatIntegrate

:ref:`mju_quatIntegrate` の導関数。

:math:`{\tt \small mju\_quatIntegrate}(q, v, h)` はインプレース回転 :math:`q \leftarrow q + v h` を実行する。
ここで :math:`q \in \mathbf{S}^3` は単位クォータニオン、 :math:`v \in \mathbf{R}^3` は3D角速度、
:math:`h \in \mathbf{R^+}` はタイムステップである。これは :math:`{\tt \small mju\_quatIntegrate}(q, s, 1.0)` と等価であり、
:math:`s` はスケーリングされた速度 :math:`s = h v` である。

:math:`{\tt \small mjd\_quatIntegrate}(v, h, D_q, D_v, D_h)` は出力 :math:`q` の入力に対するヤコビアンを計算する。
以下で :math:`\bar q` は変更前のクォータニオンを表す:

.. math::
   \begin{aligned}
      D_q &= \partial q / \partial \bar q \\
      D_v &= \partial q / \partial v \\
      D_h &= \partial q / \partial h
   \end{aligned}

導関数は :math:`h` と :math:`v` のみに依存することに注意（実際には :math:`s = h v` に依存）。
すべての出力はオプション。

*Nullable:* ``Dquat``, ``Dvel``, ``Dscale``

.. _Plugins-api:

プラグイン
^^^^^^^^^^^^^^^
.. _mjp_defaultPlugin:

`mjp_defaultPlugin <#mjp_defaultPlugin>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjp_defaultPlugin

デフォルトのプラグイン定義を設定する。

.. _mjp_registerPlugin:

`mjp_registerPlugin <#mjp_registerPlugin>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjp_registerPlugin

プラグインをグローバルに登録する。この関数はスレッドセーフである。

同一の mjpPlugin がすでに登録されている場合、この関数は何もしない。

同じ名前で非同一の mjpPlugin がすでに登録されている場合、mju_error が発生する。

2つの mjpPlugin は、すべてのメンバ関数ポインタと数値が等しく、名前と属性の文字列がすべて同一であれば
同一とみなされる。ただし、文字列への char ポインタが同じである必要はない。

.. _mjp_pluginCount:

`mjp_pluginCount <#mjp_pluginCount>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjp_pluginCount

グローバルに登録されたプラグインの数を返す。

.. _mjp_getPlugin:

`mjp_getPlugin <#mjp_getPlugin>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjp_getPlugin

名前でプラグインを検索する。slot が NULL でない場合、登録されたスロット番号も書き込む。

.. _mjp_getPluginAtSlot:

`mjp_getPluginAtSlot <#mjp_getPluginAtSlot>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjp_getPluginAtSlot

mjp_registerPlugin が返した登録済みスロット番号でプラグインを検索する。

.. _mjp_defaultResourceProvider:

`mjp_defaultResourceProvider <#mjp_defaultResourceProvider>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjp_defaultResourceProvider

デフォルトのリソースプロバイダ定義を設定する。

.. _mjp_registerResourceProvider:

`mjp_registerResourceProvider <#mjp_registerResourceProvider>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjp_registerResourceProvider

リソースプロバイダをスレッドセーフな方法でグローバルに登録する。プロバイダは、現在登録されている
プロバイダのサブプレフィックスまたはスーパープレフィックスではないプレフィックスを持つ必要がある。

成功時は 0 以上のスロット番号を返し、失敗時は -1 を返す。

.. _mjp_resourceProviderCount:

`mjp_resourceProviderCount <#mjp_resourceProviderCount>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjp_resourceProviderCount

グローバルに登録されたリソースプロバイダの数を返す。

.. _mjp_getResourceProvider:

`mjp_getResourceProvider <#mjp_getResourceProvider>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjp_getResourceProvider

リソース名に一致するプレフィックスを持つリソースプロバイダを返す。

一致しない場合、NULL を返す。

.. _mjp_getResourceProviderAtSlot:

`mjp_getResourceProviderAtSlot <#mjp_getResourceProviderAtSlot>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjp_getResourceProviderAtSlot

mjp_registerResourceProvider が返したスロット番号でリソースプロバイダを検索する。

無効なスロット番号の場合、NULL を返す。

.. _mjp_registerDecoder:

`mjp_registerDecoder <#mjp_registerDecoder>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjp_registerDecoder

デコーダをグローバルに登録する。この関数はスレッドセーフである。

同一の mjpDecoder がすでに登録されている場合、この関数は何もしない。

同じ名前で非同一の mjpDecoder がすでに登録されている場合、mju_error が発生する。

.. _mjp_defaultDecoder:

`mjp_defaultDecoder <#mjp_defaultDecoder>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjp_defaultDecoder

デフォルトのリソースデコーダ定義を設定する。

.. _mjp_findDecoder:

`mjp_findDecoder <#mjp_findDecoder>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjp_findDecoder

リソース名に一致するプレフィックスを持つリソースプロバイダを返す。

一致しない場合、NULL を返す。

.. _Thread:

スレッド
^^^^^^^^^^^^
.. _mju_threadPoolCreate:

`mju_threadPoolCreate <#mju_threadPoolCreate>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_threadPoolCreate

指定された数のスレッドが実行されるスレッドプールを作成する。

.. _mju_bindThreadPool:

`mju_bindThreadPool <#mju_bindThreadPool>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_bindThreadPool

mjData にスレッドプールを追加し、マルチスレッド使用のために構成する。

.. _mju_threadPoolEnqueue:

`mju_threadPoolEnqueue <#mju_threadPoolEnqueue>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_threadPoolEnqueue

スレッドプールにタスクをキューイングする。

.. _mju_threadPoolDestroy:

`mju_threadPoolDestroy <#mju_threadPoolDestroy>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_threadPoolDestroy

スレッドプールを破棄する。

.. _mju_defaultTask:

`mju_defaultTask <#mju_defaultTask>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_defaultTask

mjTask を初期化する。

.. _mju_taskJoin:

`mju_taskJoin <#mju_taskJoin>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_taskJoin

タスクの完了を待つ。

.. _Standardmath:

標準数学関数
^^^^^^^^^^^^^^^^^^

このセクションの「関数」は、対応するC標準ライブラリの数学関数に置換されるプリプロセッサマクロである。MuJoCoが単精度でコンパイルされた場合（現在一般には公開されていないが、内部で使用することがある）、これらのマクロは対応する単精度関数に置換される（ここでは示していない）。したがって、入出力の型はmjtNumであり、mjtNumはMuJoCoのコンパイル方法に応じてdoubleまたはfloatとして定義される。ここではこれらの関数のドキュメントは記載しない。C標準ライブラリの仕様を参照のこと。

mju_sqrt
~~~~~~~~

.. code-block:: C

   #define mju_sqrt    sqrt

mju_exp
~~~~~~~

.. code-block:: C

   #define mju_exp     exp

mju_sin
~~~~~~~

.. code-block:: C

   #define mju_sin     sin

mju_cos
~~~~~~~

.. code-block:: C

   #define mju_cos     cos

mju_tan
~~~~~~~

.. code-block:: C

   #define mju_tan     tan

mju_asin
~~~~~~~~

.. code-block:: C

   #define mju_asin    asin

mju_acos
~~~~~~~~

.. code-block:: C

   #define mju_acos    acos

mju_atan2
~~~~~~~~~

.. code-block:: C

   #define mju_atan2   atan2

mju_tanh
~~~~~~~~

.. code-block:: C

   #define mju_tanh    tanh

mju_pow
~~~~~~~

.. code-block:: C

   #define mju_pow     pow

mju_abs
~~~~~~~

.. code-block:: C

   #define mju_abs     fabs

mju_log
~~~~~~~

.. code-block:: C

   #define mju_log     log

mju_log10
~~~~~~~~~

.. code-block:: C

   #define mju_log10   log10

mju_floor
~~~~~~~~~

.. code-block:: C

   #define mju_floor   floor

mju_ceil
~~~~~~~~

.. code-block:: C

   #define mju_ceil    ceil

.. _Vectormath:

ベクトル演算
^^^^^^^^^^^^^^^^^^

.. _mju_zero3:

`mju_zero3 <#mju_zero3>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_zero3

res = 0 を設定する。

.. _mju_copy3:

`mju_copy3 <#mju_copy3>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_copy3

res = vec を設定する。

.. _mju_scl3:

`mju_scl3 <#mju_scl3>`__
~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_scl3

res = vec*scl を設定する。

.. _mju_add3:

`mju_add3 <#mju_add3>`__
~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_add3

res = vec1 + vec2 を設定する。

.. _mju_sub3:

`mju_sub3 <#mju_sub3>`__
~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_sub3

res = vec1 - vec2 を設定する。

.. _mju_addTo3:

`mju_addTo3 <#mju_addTo3>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_addTo3

res = res + vec を設定する。

.. _mju_subFrom3:

`mju_subFrom3 <#mju_subFrom3>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_subFrom3

res = res - vec を設定する。

.. _mju_addToScl3:

`mju_addToScl3 <#mju_addToScl3>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_addToScl3

res = res + vec*scl を設定する。

.. _mju_addScl3:

`mju_addScl3 <#mju_addScl3>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_addScl3

res = vec1 + vec2*scl を設定する。

.. _mju_normalize3:

`mju_normalize3 <#mju_normalize3>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_normalize3

ベクトルを正規化する。正規化前の長さを返す。

.. _mju_norm3:

`mju_norm3 <#mju_norm3>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_norm3

ベクトルの長さを返す（ベクトルを正規化しない）。

.. _mju_dot3:

`mju_dot3 <#mju_dot3>`__
~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_dot3

vec1 と vec2 の内積を返す。

.. _mju_dist3:

`mju_dist3 <#mju_dist3>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_dist3

3Dベクトル pos1 と pos2 間のデカルト距離を返す。

.. _mju_mulMatVec3:

`mju_mulMatVec3 <#mju_mulMatVec3>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_mulMatVec3

3x3行列とベクトルを乗算する: res = mat * vec。

.. _mju_mulMatTVec3:

`mju_mulMatTVec3 <#mju_mulMatTVec3>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_mulMatTVec3

転置された3x3行列とベクトルを乗算する: res = mat' * vec。

.. _mju_cross:

`mju_cross <#mju_cross>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_cross

外積を計算する: res = cross(a, b)。

.. _mju_zero4:

`mju_zero4 <#mju_zero4>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_zero4

res = 0 を設定する。

.. _mju_unit4:

`mju_unit4 <#mju_unit4>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_unit4

res = (1,0,0,0) を設定する。

.. _mju_copy4:

`mju_copy4 <#mju_copy4>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_copy4

res = vec を設定する。

.. _mju_normalize4:

`mju_normalize4 <#mju_normalize4>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_normalize4

ベクトルを正規化する。正規化前の長さを返す。

.. _mju_zero:

`mju_zero <#mju_zero>`__
~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_zero

res = 0 を設定する。

.. _mju_fill:

`mju_fill <#mju_fill>`__
~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_fill

res = val を設定する。

.. _mju_copy:

`mju_copy <#mju_copy>`__
~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_copy

res = vec を設定する。

.. _mju_sum:

`mju_sum <#mju_sum>`__
~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_sum

sum(vec)を返す。

.. _mju_L1:

`mju_L1 <#mju_L1>`__
~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_L1

L1ノルムを返す: sum(abs(vec))。

.. _mju_scl:

`mju_scl <#mju_scl>`__
~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_scl

res = vec*scl を設定する。

.. _mju_add:

`mju_add <#mju_add>`__
~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_add

res = vec1 + vec2 を設定する。

.. _mju_sub:

`mju_sub <#mju_sub>`__
~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_sub

res = vec1 - vec2 を設定する。

.. _mju_addTo:

`mju_addTo <#mju_addTo>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_addTo

res = res + vec を設定する。

.. _mju_subFrom:

`mju_subFrom <#mju_subFrom>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_subFrom

res = res - vec を設定する。

.. _mju_addToScl:

`mju_addToScl <#mju_addToScl>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_addToScl

res = res + vec*scl を設定する。

.. _mju_addScl:

`mju_addScl <#mju_addScl>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_addScl

res = vec1 + vec2*scl を設定する。

.. _mju_normalize:

`mju_normalize <#mju_normalize>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_normalize

ベクトルを正規化する。正規化前の長さを返す。

.. _mju_norm:

`mju_norm <#mju_norm>`__
~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_norm

ベクトルの長さを返す（ベクトルを正規化しない）。

.. _mju_dot:

`mju_dot <#mju_dot>`__
~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_dot

vec1 と vec2 の内積を返す。

.. _mju_mulMatVec:

`mju_mulMatVec <#mju_mulMatVec>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_mulMatVec

行列とベクトルを乗算する: res = mat * vec。

.. _mju_mulMatTVec:

`mju_mulMatTVec <#mju_mulMatTVec>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_mulMatTVec

転置行列とベクトルを乗算する: res = mat' * vec。

.. _mju_mulVecMatVec:

`mju_mulVecMatVec <#mju_mulVecMatVec>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_mulVecMatVec

正方行列と両側のベクトルを乗算する: vec1' * mat * vec2 を返す。

.. _mju_transpose:

`mju_transpose <#mju_transpose>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_transpose

行列を転置する: res = mat'。

.. _mju_symmetrize:

`mju_symmetrize <#mju_symmetrize>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_symmetrize

正方行列を対称化する :math:`R = \frac{1}{2}(M + M^T)` 。

.. _mju_eye:

`mju_eye <#mju_eye>`__
~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_eye

mat を単位行列に設定する。

.. _mju_mulMatMat:

`mju_mulMatMat <#mju_mulMatMat>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_mulMatMat

行列を乗算する: res = mat1 * mat2。

.. _mju_mulMatMatT:

`mju_mulMatMatT <#mju_mulMatMatT>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_mulMatMatT

行列を乗算する（第2引数を転置）: res = mat1 * mat2'。

.. _mju_mulMatTMat:

`mju_mulMatTMat <#mju_mulMatTMat>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_mulMatTMat

行列を乗算する（第1引数を転置）: res = mat1' * mat2。

.. _mju_sqrMatTD:

`mju_sqrMatTD <#mju_sqrMatTD>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_sqrMatTD

diagがNULLでない場合はres = mat' * diag * matを設定し、それ以外の場合はres = mat' * matを設定する。

.. _mju_transformSpatial:

`mju_transformSpatial <#mju_transformSpatial>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_transformSpatial

回転:並進形式の6D運動または力ベクトルの座標変換。
rotnew2old は 3x3 で、NULL は回転なしを意味する。flg_force は力または運動のタイプを指定する。

*Nullable:* ``rotnew2old``

.. _Sparsemath:

スパース行列演算
^^^^^^^^^^^^^^^^^^^^^^^^
.. _mju_dense2sparse:

`mju_dense2sparse <#mju_dense2sparse>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_dense2sparse

密行列からスパース行列に変換する。
 nnz は res と colind のサイズ。小さすぎる場合は1、そうでなければ0を返す。

.. _mju_sparse2dense:

`mju_sparse2dense <#mju_sparse2dense>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_sparse2dense

スパース行列から密行列に変換する。

.. _Quaternions:

クォータニオン
^^^^^^^^^^^^^^^^^^^^^

.. _mju_rotVecQuat:

`mju_rotVecQuat <#mju_rotVecQuat>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_rotVecQuat

クォータニオンによってベクトルを回転させる。

.. _mju_negQuat:

`mju_negQuat <#mju_negQuat>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_negQuat

クォータニオンの共役。反対の回転に対応する。

.. _mju_mulQuat:

`mju_mulQuat <#mju_mulQuat>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_mulQuat

クォータニオンを乗算する。

.. _mju_mulQuatAxis:

`mju_mulQuatAxis <#mju_mulQuatAxis>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_mulQuatAxis

クォータニオンと軸を乗算する。

.. _mju_axisAngle2Quat:

`mju_axisAngle2Quat <#mju_axisAngle2Quat>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_axisAngle2Quat

axisAngleをクォータニオンに変換する。

.. _mju_quat2Vel:

`mju_quat2Vel <#mju_quat2Vel>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_quat2Vel

クォータニオン（姿勢の差に対応）を3D速度に変換する。

.. _mju_subQuat:

`mju_subQuat <#mju_subQuat>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_subQuat

クォータニオンを減算し、3D速度として表現する: qb*quat(res) = qa。

.. _mju_quat2Mat:

`mju_quat2Mat <#mju_quat2Mat>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_quat2Mat

クォータニオンを3D回転行列に変換する。

.. _mju_mat2Quat:

`mju_mat2Quat <#mju_mat2Quat>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_mat2Quat

3D回転行列をクォータニオンに変換する。

.. _mju_derivQuat:

`mju_derivQuat <#mju_derivQuat>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_derivQuat

3D回転速度が与えられた場合のクォータニオンの時間導関数を計算する。

.. _mju_quatIntegrate:

`mju_quatIntegrate <#mju_quatIntegrate>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_quatIntegrate

3D角速度が与えられた場合のクォータニオンを積分する。

.. _mju_quatZ2Vec:

`mju_quatZ2Vec <#mju_quatZ2Vec>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_quatZ2Vec

z軸から指定されたベクトルへの回転を行うクォータニオンを構築する。

.. _mju_mat2Rot:

`mju_mat2Rot <#mju_mat2Rot>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_mat2Rot

入力クォータニオンを精緻化することで任意の3x3行列から3D回転を抽出する。

収束に必要な反復回数を返す。

.. _mju_euler2Quat:

`mju_euler2Quat <#mju_euler2Quat>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_euler2Quat

オイラー角の列（ラジアン）をクォータニオンに変換する。
seq[0,1,2] は 'xyzXYZ' のいずれかで、小文字/大文字は内因性/外因性回転を意味する。

.. _Poses:

姿勢
^^^^^^

.. _mju_mulPose:

`mju_mulPose <#mju_mulPose>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_mulPose

2つの姿勢を乗算する。

.. _mju_negPose:

`mju_negPose <#mju_negPose>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_negPose

姿勢の共役。反対の空間変換に対応する。

.. _mju_trnVecPose:

`mju_trnVecPose <#mju_trnVecPose>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_trnVecPose

姿勢によってベクトルを変換する。

.. _Decompositions:

分解・ソルバー
^^^^^^^^^^^^^^^^^^^^^^^^

.. _mju_cholFactor:

`mju_cholFactor <#mju_cholFactor>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_cholFactor

コレスキー分解: mat = L*L'。ランクを返す。分解は mat 内でインプレースに行われる。

.. _mju_cholSolve:

`mju_cholSolve <#mju_cholSolve>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_cholSolve

(mat*mat') * res = vec を解く。mat はコレスキー因子。

.. _mju_cholUpdate:

`mju_cholUpdate <#mju_cholUpdate>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_cholUpdate

コレスキーのランク1更新: L*L' +/- x*x'。ランクを返す。

.. _mju_cholFactorBand:

`mju_cholFactorBand <#mju_cholFactorBand>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_cholFactorBand

帯密コレスキー分解。
|br| 分解前に対角に ``diagadd + diagmul*mat_ii`` を加算する。
|br| 因子分解された対角の最小値を返す。ランク不足の場合は0を返す。

   **対称帯密行列**

   :ref:`mju_cholFactorBand` およびサブストリング "band" を含む後続の関数は、対称 `帯行列 <https://en.wikipedia.org/wiki/Band_matrix>`_ を一般化した行列に対して動作する。 *対称帯密* または「アローヘッド」行列は、対角近傍の帯に沿った非ゼロ要素と、下部の行および右側の列に密なブロックを持つ。これらの行列はコレスキー分解がフィルインを生じないという性質を持ち、そのためインプレースで効率的に実行できる。行列構造は3つの整数で定義される:

   - ``ntotal``: 対称行列の行数（列数）。
   - ``nband``: 対角の下（上）の帯の数。対角を含む。
   - ``ndense``: 下部（右側）の密な行数（列数）。

   非ゼロ要素はメモリ上に2つの連続した行優先ブロックとして格納され、下図では緑と青で色分けされている。最初のブロックはサイズ ``nband x (ntotal-ndense)`` で、対角とその下の帯を含む。2番目のブロックはサイズ ``ndense x ntotal`` で、密な部分を含む。必要な総メモリはブロックサイズの合計である。

   .. figure:: /images/APIreference/arrowhead.svg
      :width: 750px
      :align: left

   例えば、 ``nband = 3`` 、 ``ndense = 2`` 、 ``ntotal = 8`` のアローヘッド行列を考える。この例では、必要な総メモリは ``3*(8-2) + 2*8 = 34`` 個のmjtNumであり、以下のように配置される:

   .. code-block::

      0   1   2
          3   4   5
              6   7   8
                  9   10  11
                      12  13  14
                          15  16  17
              18  19  20  21  22  23  24  25
              26  27  28  29  30  31  32  33


   対角要素は ``2, 5, 8, 11, 14, 17, 24, 33`` である。
   |br| 要素 ``0, 1, 3, 25`` はメモリ上に存在するが、アクセスされることはない。

.. _mju_cholSolveBand:

`mju_cholSolveBand <#mju_cholSolveBand>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_cholSolveBand

(mat*mat')*res = vec を解く。mat は帯密コレスキー因子。

.. _mju_band2Dense:

`mju_band2Dense <#mju_band2Dense>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_band2Dense

帯行列を密行列に変換する。flg_sym>0 の場合は上三角を埋める。

.. _mju_dense2Band:

`mju_dense2Band <#mju_dense2Band>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_dense2Band

密行列を帯行列に変換する。

.. _mju_bandMulMatVec:

`mju_bandMulMatVec <#mju_bandMulMatVec>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_bandMulMatVec

帯対角行列と nvec 個のベクトルを乗算する。flg_sym>0 の場合は上三角を含める。

.. _mju_bandDiag:

`mju_bandDiag <#mju_bandDiag>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_bandDiag

帯密行列表現における対角要素 i のアドレス。

.. _mju_eig3:

`mju_eig3 <#mju_eig3>`__
~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_eig3

対称 3x3 行列の固有値分解: mat = eigvec * diag(eigval) * eigvec'。

.. _mju_boxQP:

`mju_boxQP <#mju_boxQP>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_boxQP

:math:`\tfrac{1}{2} x^T H x + x^T g \quad \text{s.t.} \quad l \le x \le u` を最小化する。ランクを返す。失敗時は-1を返す。

入力:
  ``n``           - 問題の次元数

  ``H``           - SPD行列                   ``n*n``

  ``g``           - バイアスベクトル            ``n``

  ``lower``       - 下限                       ``n``

  ``upper``       - 上限                       ``n``

  ``res``         - 解のウォームスタート         ``n``

戻り値:
  ``nfree <= n``  - 非制約部分空間のランク。失敗時は-1

出力（必須）:
  ``res``         - 解                         ``n``

  ``R``           - 部分空間のコレスキー因子     ``nfree*nfree`` 、確保サイズ: ``n*(n+7)``

出力（オプション）:
  ``index``       - 自由な次元の集合             ``nfree`` 、確保サイズ: ``n``

注意:
  ``res`` の初期値はソルバーのウォームスタートに使用される。
  ``R`` は ``n*(n+7)`` のサイズを確保する必要があるが、出力として使用されるのは ``nfree*nfree`` 個の値のみである。
  ``index`` （指定時）は ``n`` のサイズを確保する必要があるが、出力として使用されるのは ``nfree`` 個の値のみである。
  便利関数 :ref:`mju_boxQPmalloc` は必要なデータ構造を確保する。
  HとRの下三角のみがそれぞれ読み書きされる。

.. _mju_boxQPmalloc:

`mju_boxQPmalloc <#mju_boxQPmalloc>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mju_boxQPmalloc

ボックス制約二次計画法のためのヒープメモリを割り当てる。
:ref:`mju_boxQP` と同様に、 ``index`` 、 ``lower`` 、 ``upper`` はオプション。
すべてのポインタは ``mju_free()`` で解放する。

.. _Attachment:

アタッチメント
^^^^^^^^^^^^^^^^^^^^^
.. _mjs_attach:

`mjs_attach <#mjs_attach>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_attach

子を親にアタッチする。成功した場合はアタッチされた要素を返し、そうでなければ NULL を返す。

.. _AddTreeElements:

ツリー要素
^^^^^^^^^^^^^^^
.. _mjs_addBody:

`mjs_addBody <#mjs_addBody>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_addBody

ボディに子ボディを追加する。子を返す。

*Nullable:* ``def``

.. _mjs_addSite:

`mjs_addSite <#mjs_addSite>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_addSite

ボディにサイトを追加する。サイトspecを返す。

*Nullable:* ``def``

.. _mjs_addJoint:

`mjs_addJoint <#mjs_addJoint>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_addJoint

ボディにジョイントを追加する。

*Nullable:* ``def``

.. _mjs_addFreeJoint:

`mjs_addFreeJoint <#mjs_addFreeJoint>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_addFreeJoint

ボディにフリージョイントを追加する。

.. _mjs_addGeom:

`mjs_addGeom <#mjs_addGeom>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_addGeom

ボディにジオムを追加する。

*Nullable:* ``def``

.. _mjs_addCamera:

`mjs_addCamera <#mjs_addCamera>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_addCamera

ボディにカメラを追加する。

*Nullable:* ``def``

.. _mjs_addLight:

`mjs_addLight <#mjs_addLight>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_addLight

ボディにライトを追加する。

*Nullable:* ``def``

.. _mjs_addFrame:

`mjs_addFrame <#mjs_addFrame>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_addFrame

ボディにフレームを追加する。

.. _mjs_delete:

`mjs_delete <#mjs_delete>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_delete

指定された要素に対応するオブジェクトを削除する。成功時は0を返す。

.. _AddNonTreeElements:

非ツリー要素
^^^^^^^^^^^^^^^^^^
.. _mjs_addActuator:

`mjs_addActuator <#mjs_addActuator>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_addActuator

アクチュエータを追加する。

*Nullable:* ``def``

.. _mjs_addSensor:

`mjs_addSensor <#mjs_addSensor>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_addSensor

センサーを追加する。

.. _mjs_addFlex:

`mjs_addFlex <#mjs_addFlex>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_addFlex

フレックスを追加する。

.. _mjs_addPair:

`mjs_addPair <#mjs_addPair>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_addPair

接触ペアを追加する。

*Nullable:* ``def``

.. _mjs_addExclude:

`mjs_addExclude <#mjs_addExclude>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_addExclude

除外ボディペアを追加する。

.. _mjs_addEquality:

`mjs_addEquality <#mjs_addEquality>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_addEquality

等式制約を追加する。

*Nullable:* ``def``

.. _mjs_addTendon:

`mjs_addTendon <#mjs_addTendon>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_addTendon

テンドンを追加する。

*Nullable:* ``def``

.. _mjs_wrapSite:

`mjs_wrapSite <#mjs_wrapSite>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_wrapSite

テンドンを使用してサイトをラップする。

.. _mjs_wrapGeom:

`mjs_wrapGeom <#mjs_wrapGeom>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_wrapGeom

テンドンを使用してジオムをラップする。

.. _mjs_wrapJoint:

`mjs_wrapJoint <#mjs_wrapJoint>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_wrapJoint

テンドンを使用してジョイントをラップする。

.. _mjs_wrapPulley:

`mjs_wrapPulley <#mjs_wrapPulley>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_wrapPulley

テンドンを使用してプーリーをラップする。

.. _mjs_addNumeric:

`mjs_addNumeric <#mjs_addNumeric>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_addNumeric

数値を追加する。

.. _mjs_addText:

`mjs_addText <#mjs_addText>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_addText

テキストを追加する。

.. _mjs_addTuple:

`mjs_addTuple <#mjs_addTuple>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_addTuple

タプルを追加する。

.. _mjs_addKey:

`mjs_addKey <#mjs_addKey>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_addKey

キーフレームを追加する。

.. _mjs_addPlugin:

`mjs_addPlugin <#mjs_addPlugin>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_addPlugin

プラグインを追加する。

.. _mjs_addDefault:

`mjs_addDefault <#mjs_addDefault>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_addDefault

デフォルトを追加する。

*Nullable:* ``parent``

.. _AddAssets:

アセット
^^^^^^^^^^^^
.. _mjs_addMesh:

`mjs_addMesh <#mjs_addMesh>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_addMesh

メッシュを追加する。

*Nullable:* ``def``

.. _mjs_addHField:

`mjs_addHField <#mjs_addHField>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_addHField

ハイトフィールドを追加する。

.. _mjs_addSkin:

`mjs_addSkin <#mjs_addSkin>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_addSkin

スキンを追加する。

.. _mjs_addTexture:

`mjs_addTexture <#mjs_addTexture>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_addTexture

テクスチャを追加する。

.. _mjs_addMaterial:

`mjs_addMaterial <#mjs_addMaterial>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_addMaterial

マテリアルを追加する。

*Nullable:* ``def``

.. _mjs_makeMesh:

`mjs_makeMesh <#mjs_makeMesh>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_makeMesh

メッシュの頂点と法線を設定する。

.. _FindAndGetUtilities:

検索・取得ユーティリティ
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.. _mjs_getSpec:

`mjs_getSpec <#mjs_getSpec>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_getSpec

ボディからspecを取得する。

.. _mjs_findSpec:

`mjs_findSpec <#mjs_findSpec>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_findSpec

名前でspec（モデルアセット）を検索する。

.. _mjs_findBody:

`mjs_findBody <#mjs_findBody>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_findBody

名前でspec内のボディを検索する。

.. _mjs_findElement:

`mjs_findElement <#mjs_findElement>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_findElement

名前でspec内の要素を検索する。

.. _mjs_findChild:

`mjs_findChild <#mjs_findChild>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_findChild

名前で子ボディを検索する。

.. _mjs_getParent:

`mjs_getParent <#mjs_getParent>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_getParent

親ボディを取得する。

.. _mjs_getFrame:

`mjs_getFrame <#mjs_getFrame>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_getFrame

親フレームを取得する。

.. _mjs_findFrame:

`mjs_findFrame <#mjs_findFrame>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_findFrame

名前でフレームを検索する。

.. _mjs_getDefault:

`mjs_getDefault <#mjs_getDefault>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_getDefault

要素に対応するデフォルトを取得する。

.. _mjs_findDefault:

`mjs_findDefault <#mjs_findDefault>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_findDefault

クラス名でモデル内のデフォルトを検索する。

.. _mjs_getSpecDefault:

`mjs_getSpecDefault <#mjs_getSpecDefault>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_getSpecDefault

モデルからグローバルデフォルトを取得する。

.. _mjs_getId:

`mjs_getId <#mjs_getId>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_getId

要素のIDを取得する。

.. _mjs_firstChild:

`mjs_firstChild <#mjs_firstChild>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_firstChild

指定された型のボディの最初の子を返す。recurse が非ゼロの場合、ボディのサブツリーも検索する。

.. _mjs_nextChild:

`mjs_nextChild <#mjs_nextChild>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_nextChild

同じ型のボディの次の子を返す。子が最後の場合は NULL を返す。

recurse が非ゼロの場合、ボディのサブツリーも検索する。

.. _mjs_firstElement:

`mjs_firstElement <#mjs_firstElement>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_firstElement

選択された型のspecの最初の要素を返す。

.. _mjs_nextElement:

`mjs_nextElement <#mjs_nextElement>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_nextElement

specの次の要素を返す。要素が最後の場合は NULL を返す。

.. _mjs_getWrapTarget:

`mjs_getWrapTarget <#mjs_getWrapTarget>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_getWrapTarget

テンドンパス内のラップされた要素を取得する。

.. _mjs_getWrapSideSite:

`mjs_getWrapSideSite <#mjs_getWrapSideSite>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_getWrapSideSite

テンドンパス内のラップされた要素のサイドサイトを取得する。存在しない場合は nullptr。

.. _mjs_getWrapDivisor:

`mjs_getWrapDivisor <#mjs_getWrapDivisor>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_getWrapDivisor

プーリーをラップする mjsWrap の除数を取得する。

.. _mjs_getWrapCoef:

`mjs_getWrapCoef <#mjs_getWrapCoef>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_getWrapCoef

ジョイントをラップする mjsWrap の係数を取得する。

.. _AttributeSetters:

属性セッター
^^^^^^^^^^^^^^^^^^
.. _mjs_setName:

`mjs_setName <#mjs_setName>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_setName

要素の名前を設定する。成功時は0を返す。

.. _mjs_setBuffer:

`mjs_setBuffer <#mjs_setBuffer>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_setBuffer

バッファをコピーする。

.. _mjs_setString:

`mjs_setString <#mjs_setString>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_setString

テキストを文字列にコピーする。

.. _mjs_setStringVec:

`mjs_setStringVec <#mjs_setStringVec>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_setStringVec

テキストをエントリに分割し、文字列ベクトルにコピーする。

.. _mjs_setInStringVec:

`mjs_setInStringVec <#mjs_setInStringVec>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_setInStringVec

文字列ベクトルのエントリを設定する。

.. _mjs_appendString:

`mjs_appendString <#mjs_appendString>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_appendString

文字列ベクトルにテキストエントリを追加する。

.. _mjs_setInt:

`mjs_setInt <#mjs_setInt>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_setInt

int配列をベクトルにコピーする。

.. _mjs_appendIntVec:

`mjs_appendIntVec <#mjs_appendIntVec>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_appendIntVec

配列のベクトルにint配列を追加する。

.. _mjs_setFloat:

`mjs_setFloat <#mjs_setFloat>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_setFloat

float配列をベクトルにコピーする。

.. _mjs_appendFloatVec:

`mjs_appendFloatVec <#mjs_appendFloatVec>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_appendFloatVec

配列のベクトルにfloat配列を追加する。

.. _mjs_setDouble:

`mjs_setDouble <#mjs_setDouble>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_setDouble

double配列をベクトルにコピーする。

.. _mjs_setPluginAttributes:

`mjs_setPluginAttributes <#mjs_setPluginAttributes>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_setPluginAttributes

プラグイン属性を設定する。

.. _AttributeGetters:

属性ゲッター
^^^^^^^^^^^^^^^^^^
.. _mjs_getName:

`mjs_getName <#mjs_getName>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_getName

要素の名前を取得する。

.. _mjs_getString:

`mjs_getString <#mjs_getString>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_getString

文字列の内容を取得する。

.. _mjs_getDouble:

`mjs_getDouble <#mjs_getDouble>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_getDouble

double配列の内容と、オプションでそのサイズを取得する。

*Nullable:* ``size``

.. _mjs_getWrapNum:

`mjs_getWrapNum <#mjs_getWrapNum>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_getWrapNum

テンドンが巻き付く要素の数を取得する。

.. _mjs_getWrap:

`mjs_getWrap <#mjs_getWrap>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_getWrap

テンドンパスの位置iにあるmjsWrap要素を取得する。

.. _mjs_getPluginAttributes:

`mjs_getPluginAttributes <#mjs_getPluginAttributes>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_getPluginAttributes

プラグイン属性を取得する。

.. _SpecUtilities:

Specユーティリティ
^^^^^^^^^^^^^^^^^^^^^^^^^
.. _mjs_setDefault:

`mjs_setDefault <#mjs_setDefault>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_setDefault

要素のデフォルトを設定する。

.. _mjs_setFrame:

`mjs_setFrame <#mjs_setFrame>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_setFrame

要素の囲むフレームを設定する。成功時に0を返す。

.. _mjs_resolveOrientation:

`mjs_resolveOrientation <#mjs_resolveOrientation>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_resolveOrientation

代替の姿勢表現をクォータニオンに変換する。エラーがあればそれを返す。

.. _mjs_bodyToFrame:

`mjs_bodyToFrame <#mjs_bodyToFrame>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_bodyToFrame

ボディをフレームに変換する。

.. _mjs_setUserValue:

`mjs_setUserValue <#mjs_setUserValue>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_setUserValue

ユーザーペイロードを設定する。指定されたキーの既存値がある場合は上書きする。

.. _mjs_setUserValueWithCleanup:

`mjs_setUserValueWithCleanup <#mjs_setUserValueWithCleanup>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_setUserValueWithCleanup

ユーザーペイロードを設定する。指定されたキーの既存値がある場合は上書きする。
このバージョンはmjs_setUserValueとは異なり、ユーザーペイロードが削除される
ときに呼び出されるクリーンアップ関数を受け取る。

.. _mjs_getUserValue:

`mjs_getUserValue <#mjs_getUserValue>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_getUserValue

ユーザーペイロードを返す。見つからない場合はNULLを返す。

.. _mjs_deleteUserValue:

`mjs_deleteUserValue <#mjs_deleteUserValue>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_deleteUserValue

ユーザーペイロードを削除する。

.. _mjs_sensorDim:

`mjs_sensorDim <#mjs_sensorDim>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_sensorDim

センサーの次元数を返す。

.. _ElementInitialization:

要素の初期化
^^^^^^^^^^^^^^^^^^^^^^
.. _mjs_defaultSpec:

`mjs_defaultSpec <#mjs_defaultSpec>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_defaultSpec

Specのデフォルト属性。

.. _mjs_defaultOrientation:

`mjs_defaultOrientation <#mjs_defaultOrientation>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_defaultOrientation

姿勢のデフォルト属性。

.. _mjs_defaultBody:

`mjs_defaultBody <#mjs_defaultBody>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_defaultBody

ボディのデフォルト属性。

.. _mjs_defaultFrame:

`mjs_defaultFrame <#mjs_defaultFrame>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_defaultFrame

フレームのデフォルト属性。

.. _mjs_defaultJoint:

`mjs_defaultJoint <#mjs_defaultJoint>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_defaultJoint

ジョイントのデフォルト属性。

.. _mjs_defaultGeom:

`mjs_defaultGeom <#mjs_defaultGeom>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_defaultGeom

ジオムのデフォルト属性。

.. _mjs_defaultSite:

`mjs_defaultSite <#mjs_defaultSite>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_defaultSite

サイトのデフォルト属性。

.. _mjs_defaultCamera:

`mjs_defaultCamera <#mjs_defaultCamera>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_defaultCamera

カメラのデフォルト属性。

.. _mjs_defaultLight:

`mjs_defaultLight <#mjs_defaultLight>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_defaultLight

ライトのデフォルト属性。

.. _mjs_defaultFlex:

`mjs_defaultFlex <#mjs_defaultFlex>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_defaultFlex

フレックスのデフォルト属性。

.. _mjs_defaultMesh:

`mjs_defaultMesh <#mjs_defaultMesh>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_defaultMesh

メッシュのデフォルト属性。

.. _mjs_defaultHField:

`mjs_defaultHField <#mjs_defaultHField>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_defaultHField

ハイトフィールドのデフォルト属性。

.. _mjs_defaultSkin:

`mjs_defaultSkin <#mjs_defaultSkin>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_defaultSkin

スキンのデフォルト属性。

.. _mjs_defaultTexture:

`mjs_defaultTexture <#mjs_defaultTexture>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_defaultTexture

テクスチャのデフォルト属性。

.. _mjs_defaultMaterial:

`mjs_defaultMaterial <#mjs_defaultMaterial>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_defaultMaterial

マテリアルのデフォルト属性。

.. _mjs_defaultPair:

`mjs_defaultPair <#mjs_defaultPair>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_defaultPair

ペアのデフォルト属性。

.. _mjs_defaultEquality:

`mjs_defaultEquality <#mjs_defaultEquality>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_defaultEquality

等式制約のデフォルト属性。

.. _mjs_defaultTendon:

`mjs_defaultTendon <#mjs_defaultTendon>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_defaultTendon

テンドンのデフォルト属性。

.. _mjs_defaultActuator:

`mjs_defaultActuator <#mjs_defaultActuator>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_defaultActuator

アクチュエータのデフォルト属性。

.. _mjs_defaultSensor:

`mjs_defaultSensor <#mjs_defaultSensor>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_defaultSensor

センサーのデフォルト属性。

.. _mjs_defaultNumeric:

`mjs_defaultNumeric <#mjs_defaultNumeric>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_defaultNumeric

数値のデフォルト属性。

.. _mjs_defaultText:

`mjs_defaultText <#mjs_defaultText>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_defaultText

テキストのデフォルト属性。

.. _mjs_defaultTuple:

`mjs_defaultTuple <#mjs_defaultTuple>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_defaultTuple

タプルのデフォルト属性。

.. _mjs_defaultKey:

`mjs_defaultKey <#mjs_defaultKey>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_defaultKey

キーフレームのデフォルト属性。

.. _mjs_defaultPlugin:

`mjs_defaultPlugin <#mjs_defaultPlugin>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_defaultPlugin

プラグインのデフォルト属性。

.. _ElementCasting:

要素のキャスト
^^^^^^^^^^^^^^^^^^^^^
.. _mjs_asBody:

`mjs_asBody <#mjs_asBody>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_asBody

要素をmjsBodyに安全にキャストする。要素がmjsBodyでない場合はNULLを返す。

.. _mjs_asGeom:

`mjs_asGeom <#mjs_asGeom>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_asGeom

要素をmjsGeomに安全にキャストする。要素がmjsGeomでない場合はNULLを返す。

.. _mjs_asJoint:

`mjs_asJoint <#mjs_asJoint>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_asJoint

要素をmjsJointに安全にキャストする。要素がmjsJointでない場合はNULLを返す。

.. _mjs_asSite:

`mjs_asSite <#mjs_asSite>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_asSite

要素をmjsSiteに安全にキャストする。要素がmjsSiteでない場合はNULLを返す。

.. _mjs_asCamera:

`mjs_asCamera <#mjs_asCamera>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_asCamera

要素をmjsCameraに安全にキャストする。要素がmjsCameraでない場合はNULLを返す。

.. _mjs_asLight:

`mjs_asLight <#mjs_asLight>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_asLight

要素をmjsLightに安全にキャストする。要素がmjsLightでない場合はNULLを返す。

.. _mjs_asFrame:

`mjs_asFrame <#mjs_asFrame>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_asFrame

要素をmjsFrameに安全にキャストする。要素がmjsFrameでない場合はNULLを返す。

.. _mjs_asActuator:

`mjs_asActuator <#mjs_asActuator>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_asActuator

要素をmjsActuatorに安全にキャストする。要素がmjsActuatorでない場合はNULLを返す。

.. _mjs_asSensor:

`mjs_asSensor <#mjs_asSensor>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_asSensor

要素をmjsSensorに安全にキャストする。要素がmjsSensorでない場合はNULLを返す。

.. _mjs_asFlex:

`mjs_asFlex <#mjs_asFlex>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_asFlex

要素をmjsFlexに安全にキャストする。要素がmjsFlexでない場合はNULLを返す。

.. _mjs_asPair:

`mjs_asPair <#mjs_asPair>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_asPair

要素をmjsPairに安全にキャストする。要素がmjsPairでない場合はNULLを返す。

.. _mjs_asEquality:

`mjs_asEquality <#mjs_asEquality>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_asEquality

要素をmjsEqualityに安全にキャストする。要素がmjsEqualityでない場合はNULLを返す。

.. _mjs_asExclude:

`mjs_asExclude <#mjs_asExclude>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_asExclude

要素をmjsExcludeに安全にキャストする。要素がmjsExcludeでない場合はNULLを返す。

.. _mjs_asTendon:

`mjs_asTendon <#mjs_asTendon>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_asTendon

要素をmjsTendonに安全にキャストする。要素がmjsTendonでない場合はNULLを返す。

.. _mjs_asNumeric:

`mjs_asNumeric <#mjs_asNumeric>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_asNumeric

要素をmjsNumericに安全にキャストする。要素がmjsNumericでない場合はNULLを返す。

.. _mjs_asText:

`mjs_asText <#mjs_asText>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_asText

要素をmjsTextに安全にキャストする。要素がmjsTextでない場合はNULLを返す。

.. _mjs_asTuple:

`mjs_asTuple <#mjs_asTuple>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_asTuple

要素をmjsTupleに安全にキャストする。要素がmjsTupleでない場合はNULLを返す。

.. _mjs_asKey:

`mjs_asKey <#mjs_asKey>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_asKey

要素をmjsKeyに安全にキャストする。要素がmjsKeyでない場合はNULLを返す。

.. _mjs_asMesh:

`mjs_asMesh <#mjs_asMesh>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_asMesh

要素をmjsMeshに安全にキャストする。要素がmjsMeshでない場合はNULLを返す。

.. _mjs_asHField:

`mjs_asHField <#mjs_asHField>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_asHField

要素をmjsHFieldに安全にキャストする。要素がmjsHFieldでない場合はNULLを返す。

.. _mjs_asSkin:

`mjs_asSkin <#mjs_asSkin>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_asSkin

要素をmjsSkinに安全にキャストする。要素がmjsSkinでない場合はNULLを返す。

.. _mjs_asTexture:

`mjs_asTexture <#mjs_asTexture>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_asTexture

要素をmjsTextureに安全にキャストする。要素がmjsTextureでない場合はNULLを返す。

.. _mjs_asMaterial:

`mjs_asMaterial <#mjs_asMaterial>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_asMaterial

要素をmjsMaterialに安全にキャストする。要素がmjsMaterialでない場合はNULLを返す。

.. _mjs_asPlugin:

`mjs_asPlugin <#mjs_asPlugin>`__
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. mujoco-include:: mjs_asPlugin

要素をmjsPluginに安全にキャストする。要素がmjsPluginでない場合はNULLを返す。

