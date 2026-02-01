============================
XMLリファレンス
============================

はじめに
--------

本章は、MuJoCo で使用される MJCF モデリング言語のリファレンスマニュアルです。


.. _CSchema:

XMLスキーマ
~~~~~~~~~~~

以下のドロップダウンは、MJCF における XML 要素とその属性をまとめたものです。右側のメニューは要素へのリンクであり、下記のドロップダウンは属性へのリンクです。MJCF におけるすべての情報は、要素と属性を通じて入力されることに注意してください。要素内のテキストコンテンツは使用されません。存在する場合、パーサーはそれを無視します。

.. only:: html

   各要素名の右側にあるアイコンは、以下の意味を持ちます:

   .. list-table::
      :widths: 6 94

      * - |!|
        - 必須要素、1回のみ出現可能
      * - |R|
        - オプション要素、再帰的に複数回出現可能
      * - |?|
        - オプション要素、1回のみ出現可能
      * - |m|
        - オプション要素、複数回出現可能（デフォルトケース、アイコンなし）

   |br|

   .. container:: schema-small

      .. include:: XMLschema.rst

.. only:: latex

   .. note::
      XMLスキーマテーブルは、本ドキュメントのHTML版でのみ利用可能です。


.. _CType:

属性の型
~~~~~~~~

| 各属性には、パーサーによって強制されるデータ型があります。利用可能なデータ型は以下の通りです:

========= ===================================================================================================
string    任意の文字列。通常はファイル名またはモデル要素のユーザー定義名を指定します。
int(N)    N個の整数の配列。Nが省略された場合は1になります。
real(N)   N個の実数値の配列。Nが省略された場合は1になります。
[...]     キーワード属性。有効なキーワードのリストは角括弧内に記載されます。
========= ===================================================================================================

|
| 配列型属性の場合、以下のリファレンスドキュメントで特に指定されていない限り、配列の長さはパーサーによって強制されます。
| さらに、データ型を持つことに加えて、属性は必須またはオプションになります。オプション属性は、内部デフォルト値を持つ場合と持たない場合があります。内部デフォルト値を持たないオプション属性は、特殊な未定義状態で初期化されます。この状態は、XMLで入力できる有効な設定とは異なります。このメカニズムにより、コンパイラーは、ユーザーが明示的またはデフォルトを通じて属性を「触った」かどうかを判断し、適切なアクションを実行できます。一部の属性には内部デフォルト値（通常は0）がありますが、実際にはコンパイラーによって許可されていません。このような属性が特定のコンテキストで関連する場合、許可された値に設定する必要があります。

+-------------+--------------------------------------------------------------------------------------------------+
| required    | 属性はパーサーによって必須です。存在しない場合、パーサーはエラーを生成します。                   |
+-------------+--------------------------------------------------------------------------------------------------+
| optional    | 属性はオプションです。内部デフォルト値はありません。属性は未定義状態で初期化されます。           |
+-------------+--------------------------------------------------------------------------------------------------+
| "..."       | 属性はオプションです。内部デフォルト値は引用符で示されます。                                     |
+-------------+--------------------------------------------------------------------------------------------------+


以下のリファレンスドキュメントでは、属性名は太字で表示され、その後にデータ型、その後に内部デフォルト値（存在する場合）を含む必須/オプションのステータスが続きます。例えば、angle属性は、値が「radian」または「degree」になるキーワード属性です。これはオプション属性であり、内部デフォルト値は「degree」です。したがって、リファレンスドキュメントでは次のように表示されます:

:at:`angle`: :at-val:`[radian, degree], "degree"`
   .. raw:: html

      <p style="display: none"></p>


.. _Reference:

MJCFリファレンス
----------------

MJCFファイルには、一意のトップレベル要素 :ref:`mujoco <mujoco>` があります。次のレベルの要素は *セクション* と呼ばれます。これらはすべてオプションです。一部のセクションは単にグループ化のために使用され、属性を持ちません。セクションは繰り返すことができ、 :ref:`include <include>` 要素を介したモデルのマージを容易にします。要素内の属性の *順序* は任意です。親要素内の子要素の順序も任意ですが、4つの例外があります:

-  :ref:`body <body>` 内の :ref:`joint <body-joint>` 要素の順序は重要です。ジョイント変換は順番に実行されるためです。
-  :ref:`spatial <tendon-spatial>` テンドン内の要素の順序は重要です。テンドンが通過または巻き付くオブジェクトのシーケンスを決定するためです。
-  同じ属性が異なる値に複数回設定される場合、繰り返しセクションの順序が重要です。その場合、最後の設定がモデル全体に適用されます。
-  同じデフォルトクラス内の複数のアクチュエーターショートカットの順序は重要です。各ショートカットは、そのデフォルトクラス内の単一の :ref:`general <actuator-general>` 要素の属性を設定し、以前の設定を上書きするためです。

本章の残りの部分では、すべての有効な MJCF 要素とその属性について説明します。一部の要素は複数のコンテキストで使用でき、その場合、その意味は親要素に依存します。そのため、以下のドキュメントでは常に親を接頭辞として表示します。

.. _meta-element:

メタ要素
~~~~~~~~

これらの要素は、厳密には低レベルの MJCF フォーマット定義の一部ではなく、コンパイラーにモデルに対して何らかの操作を実行するよう指示するものです。メタ要素の一般的な特性は、XMLを保存すると、モデルから消えることです。現在、MJCFには6つのメタ要素があります:

- :ref:`include<include>`, :ref:`frame<frame>`, および :ref:`replicate<replicate>` は、スキーマの外にあります。
- :ref:`composite<body-composite>`, :ref:`flexcomp<body-flexcomp>`, および :ref:`attach<body-attach>` は、スキーマの一部ですが、他の MJCF 要素をプロシージャルに生成するために機能します。

.. _frame:

**frame** |R|
^^^^^^^^^^^^^

frame メタ要素は、キネマティックツリー（ :ref:`worldbody<body>` の下）内の任意の要素グループをラップできる純粋な座標変換です。コンパイル後、frame 要素は消え、その変換は直接の子要素に蓄積されます。frame メタ要素の属性は :ref:`下記<body-frame>` に記載されています。

.. collapse:: Usage example of frame

   このモデルをロードして保存すると:

   .. code-block:: xml

      <mujoco>
        <worldbody>
          <frame quat="0 0 1 0">
             <geom name="Alice" quat="0 1 0 0" size="1"/>
          </frame>

          <frame pos="0 1 0">
            <geom name="Bob" pos="0 1 0" size="1"/>
            <body name="Carl" pos="1 0 0">
              ...
            </body>
          </frame>
        </worldbody>
      </mujoco>

   次のモデルになります:

   .. code-block:: xml

      <mujoco>
        <worldbody>
          <geom name="Alice" quat="0 0 0 1" size="1"/>
          <geom name="Bob" pos="0 2 0" size="1"/>
          <body name="Carl" pos="1 1 0">
            ...
          </body>
        </worldbody>
      </mujoco>

   保存されたモデルでは、frame 要素は消えていますが、その変換は子要素の変換と蓄積されていることに注意してください。

.. _replicate:

**replicate** |R|
^^^^^^^^^^^^^^^^^

replicate 要素は、囲まれたキネマティックツリー要素を増分的な並進および回転オフセットで複製し、名前の衝突を避けるために名前空間サフィックスを追加します。追加されるサフィックス文字列は、 ``[0...count-1]`` の範囲の整数であり、合計要素数を表すために必要な最小桁数を持ちます（つまり、200回複製する場合、サフィックスは ``000, 001, ...`` などになります）。すべての参照要素は自動的に複製され、適切に名前空間化されます。replicate を使用したモデルの詳細な例は、 `model/replicate/ <https://github.com/google-deepmind/mujoco/tree/main/model/replicate>`__ ディレクトリにあります。

replicate を使用する場合、 :ref:`keyframes<keyframe>` に関するいくつかの注意事項があります。 :ref:`mjs_attach` を使用して囲まれたキネマティックツリーを複数回自己アタッチするため、このツリーにさらに :ref:`attach<body-attach>` 要素が含まれている場合、キーフレームは :ref:`replicate<replicate>` によって複製も名前空間化もされませんが、最も内側の :ref:`mjs_attach` の呼び出しによって一度だけアタッチされ、名前空間化されます。 :ref:`attachment<meAttachment>` で説明されている制限を参照してください。

.. _replicate-count:

:at:`count`: :at-val:`int, required`
   レプリカの数。正の値である必要があります。

.. _replicate-sep:

:at:`sep`: :at-val:`string, optional`
   名前空間セパレーター。このオプションの文字列は、名前空間サフィックス文字列の前に追加されます。ネストされた replicate 要素の場合、最も内側の名前空間サフィックスが最初に追加されることに注意してください。

.. _replicate-offset:

:at:`offset`: :at-val:`real(3), optional`
   3つの座標軸に沿った並進オフセット。一般に、オフセットのフレームは前のレプリカに対するものですが、最初のレプリカは replicate 要素の親に対するものです。回転がない場合、これらの値は常に replicate 要素の親のフレーム内にあります。

.. _replicate-euler:

:at:`euler`: :at-val:`real(3), optional`
   連続する2つのレプリカ間の3つの座標軸周りの回転角度。角度の単位と回転シーケンスは、グローバルな :ref:`angle<compiler-angle>` および :ref:`eulerseq<compiler-eulerseq>` の設定に従います。回転は常に前のレプリカのフレームに対するものであるため、合計回転は累積的です。

.. collapse:: Usage example of replicate

   このモデルをロードして保存すると:

   .. code-block:: xml

      <mujoco>
        <worldbody>
          <replicate count="2" offset="0 1 0" euler="90 0 0">
            <replicate count="2" sep="-" offset="1 0 0" euler="0 90 0">
              <geom name="Alice" size=".1"/>
            </replicate>
          </replicate>
        </worldbody>

        <sensor>
          <accelerometer name="Bob" site="Alice"/>
        </sensor>
      </mujoco>

   次のモデルになります:

   .. code-block:: xml

      <mujoco>
        <worldbody>
          <geom name="Alice-00" size="0.1"/>
          <geom name="Alice-10" size="0.1" pos="1 0 0" quat="1 0 1 0"/>
          <geom name="Alice-01" size="0.1" pos="0 1 0" quat="1 1 0 0"/>
          <geom name="Alice-11" size="0.1" pos="1 1 0" quat="0.5 0.5 0.5 0.5"/>
        </worldbody>

        <sensor>
          <accelerometer name="Bob-00" site="Alice-00"/>
          <accelerometer name="Bob-10" site="Alice-10"/>
          <accelerometer name="Bob-01" site="Alice-01"/>
          <accelerometer name="Bob-11" site="Alice-11"/>
        </sensor>
      </mujoco>

.. _include:

**include** |m|
^^^^^^^^^^^^^^^

この要素は厳密には MJCF に属していません。代わりに、解析前に複数の XMLファイルを単一のドキュメントオブジェクトモデル（DOM）に組み立てるために使用されるメタ要素です。含まれるファイルは、一意のトップレベル要素を持つ有効な XML ファイルである必要があります。このトップレベル要素はパーサーによって削除され、その下の要素が :el:`include` 要素の位置に挿入されます。この手順の結果として、少なくとも1つの要素が挿入される必要があります。 :el:`include` 要素は、MJCF ファイル内で XML 要素が期待される場所であればどこでも使用できます。ネストされたインクルードは許可されていますが、特定の XML ファイルはモデル全体で最大1回だけ含めることができます。すべての含まれる XML ファイルが単一の DOMに組み立てられた後、有効な MJCF モデルに対応する必要があります。それ以外については、インクルードをどのように使用するか、また必要に応じて大きなファイルをモジュール化する方法はユーザー次第です。

:at:`file`: :at-val:`string, required`
   含めるXMLファイルの名前。ファイルの場所は、メイン MJCF ファイルのディレクトリからの相対パスです。ファイルが同じディレクトリにない場合は、相対パスを前に付ける必要があります。

.. admonition:: Prefer attach to include
   :class: note

   :ref:`include<include>` の一部の使用ケースは依然として有効ですが、該当する場合は :ref:`attach<body-attach>` 要素を使用することをお勧めします。


.. _mujoco:

**mujoco** |!|
~~~~~~~~~~~~~~

MJCF モデルファイルとしてXMLファイルを識別する、一意のトップレベル要素。

.. _mujoco-model:

:at:`model`: :at-val:`string, "MuJoCo Model"`
   モデルの名前。この名前は :ref:`simulate.cc <saSimulate>` のタイトルバーに表示される。



.. _option:

**option** |m|
~~~~~~~~~~~~~~

この要素は、mjModel の mjModel.opt フィールドに含まれる低レベル構造体 mjOption と一対一で対応している。これらはシミュレーションオプションであり、コンパイル処理には一切影響しない。単に低レベルモデルにコピーされるだけである。mjOption は実行時にユーザーが変更できるが、それでもXMLを通じて適切に調整しておくことが推奨される。

.. _option-timestep:

:at:`timestep`: :at-val:`real, "0.002"`
   シミュレーションのタイムステップ（秒）。これは、すべての物理シミュレーションに内在する速度と精度のトレードオフに影響する最も重要なパラメータである。小さい値にするとより良い精度と安定性が得られる。リアルタイム性能を達成するには、タイムステップをステップごとのCPU時間よりも大きく設定する必要がある（RK4積分器を使用する場合は4倍大きく設定する）。CPU時間は内部タイマーで測定される。タイムステップを調整する際には監視する必要がある。MuJoCo はほとんどのロボットシステムをリアルタイムよりずっと速くシミュレートできるが、多くの浮遊ボディを持つモデル（結果として多くの接触が発生する）は計算量が大きい。安定性はタイムステップだけでなく :ref:`CSolver` によっても決まることを覚えておくこと。特に柔らかい制約ではより大きなタイムステップでシミュレートできる。難しいモデルを微調整する際には、両方の設定を同時に試すことが推奨される。最適化関連のアプリケーションでは、リアルタイムでは不十分で、シミュレーションをできるだけ高速に実行することが望ましい。その場合、タイムステップはできるだけ大きくすべきである。

.. _option-impratio:

:at:`impratio`: :at-val:`real, "1"`
   この属性は、楕円摩擦円錐における摩擦制約対法線制約のインピーダンス比を決定する。solimp の設定はすべての接触次元に対して単一のインピーダンス値を決定し、それがこの属性で調整される。1 より大きい設定にすると、摩擦力が法線力よりも「硬く」なり、実際の摩擦係数を増やすことなく滑りを防ぐ全体的な効果がある。角錐摩擦円錐の場合、状況はより複雑である。角錐近似は各基底ベクトル内で法線方向と摩擦方向を混合するため、角錐円錐では高い impratio 値を使用することは推奨されない。

.. _option-gravity:

:at:`gravity`: :at-val:`real(3), "0 0 -9.81"`
   重力加速度ベクトル。デフォルトのワールド座標系では、Z軸が上を向いている。MuJoCo GUI はこの慣例を基に構成されている（カメラと摂動コマンドの両方がこれを基にしている）ので、この慣例から逸脱することは推奨されない。

.. _option-wind:

:at:`wind`: :at-val:`real(3), "0 0 0"`
   媒質（すなわち風）の速度ベクトル。このベクトルは各ボディの3次元並進速度から減算され、その結果がボディに作用する粘性力、揚力、抗力の計算に使用される。計算の章の :ref:`Passive forces <gePassive>` を参照。これらの力の大きさは、次の2つの属性の値に比例する。


.. _option-magnetic:

:at:`magnetic`: :at-val:`real(3), "0 -0.5 0"`
   グローバル磁束。このベクトルは磁力計センサーで使用される。磁力計センサーはサイトとして定義され、サイト位置での磁束をサイトフレームで表現した値を返す。

.. _option-density:

:at:`density`: :at-val:`real, "0"`
   媒質の密度。質量と慣性を推測するために使用されるジオム密度と混同しないこと。このパラメータは揚力と抗力をシミュレートするために使用される。これらの力は速度の2乗に比例する。SI単位では、空気の密度は温度にもよるがおよそ1.2、水の密度はおよそ1000である。density を 0 に設定すると揚力と抗力が無効になる。

.. _option-viscosity:

:at:`viscosity`: :at-val:`real, "0"`
   媒質の粘度。このパラメータは粘性力をシミュレートするために使用される。粘性力は速度に線形に比例する。SI単位では、空気の粘度は温度にもよるがおよそ0.00002、水の粘度はおよそ0.0009である。viscosity を 0 に設定すると粘性力が無効になる。デフォルトのオイラー :ref:`integrator <geIntegration>` は、ジョイントの減衰を陰的に処理する。これにより安定性と精度が向上する。しかし、ボディ粘性については現在このような処理を行っていない。したがって、目的が単に減衰シミュレーションを作成すること（粘性の特定の効果をモデル化するのではなく）であれば、ボディ粘性を使用するよりもジョイント減衰を使用するか、 :at:`implicit` または :at:`implicitfast` 積分器に切り替えることを推奨する。

.. _option-o_margin:

:at:`o_margin`: :at-val:`real, "0"`
   この属性は、 :ref:`Contact override <COverride>` が有効な場合に、すべてのアクティブな接触ペアの margin パラメータを置き換える。それ以外の場合、MuJoCo は接触ペアの生成方法に応じて :ref:`geom<body-geom>` または :ref:`pair<contact-pair>` の要素固有の margin 属性を使用する。計算の章の :ref:`Collision` も参照。関連する gap パラメータにはグローバルオーバーライドがない。

.. _option-o_solref:
.. _option-o_solimp:
.. _option-o_friction:

:at:`o_solref`, :at:`o_solimp`, :at:`o_friction`
   これらの属性は、contact override が有効な場合に、すべてのアクティブな接触ペアの solref、solimp、friction パラメータを置き換える。詳細は :ref:`CSolver` を参照。

.. _option-integrator:

:at:`integrator`: :at-val:`[Euler, RK4, implicit, implicitfast], "Euler"`
   この属性は、使用する数値 :ref:`integrator <geIntegration>` を選択する。現在利用可能な積分器は、半陰的オイラー法、固定ステップ4次ルンゲ・クッタ法、速度に関する陰的オイラー法、そして :at:`implicitfast` （コリオリ力と遠心力の項を省略）である。詳細は :ref:`Numerical Integration<geIntegration>` を参照。

.. _option-cone:

:at:`cone`: :at-val:`[pyramidal, elliptic], "pyramidal"`
   接触摩擦円錐のタイプ。楕円錐は物理的現実をより良くモデル化するが、角錐はソルバーを高速かつロバストにすることがある。

.. _option-jacobian:

:at:`jacobian`: :at-val:`[dense, sparse, auto], "auto"`
   制約ヤコビアンとそれから計算される行列のタイプ。auto は、自由度が60以下の場合は dense に、60を超える場合は sparse に解決される。

.. _option-solver:

:at:`solver`: :at-val:`[PGS, CG, Newton], "Newton"`
   この属性は、計算の章で説明されている制約ソルバー :ref:`algorithms <soAlgorithms>` の1つを選択する。ソルバー選択とパラメータ調整のガイドラインは、上記の :ref:`Algorithms <CAlgorithms>` セクションで利用可能である。

.. _option-iterations:

:at:`iterations`: :at-val:`int, "100"`
   制約ソルバーの最大反復回数。 :ref:`flag <option-flag>` の warmstart 属性が有効な場合（デフォルト）、より少ない反復で正確な結果が得られる。多くの相互作用する制約を持つ大規模で複雑なシステムでは、より多くの反復が必要になる。なお、mjData.solver にはソルバーの収束に関する統計情報が含まれており、プロファイラーにも表示される。

.. _option-tolerance:

:at:`tolerance`: :at-val:`real, "1e-8"`
   反復ソルバーの早期終了に使用される許容閾値。PGS の場合、閾値は2つの反復間のコスト改善に適用される。CG と Newton の場合、コスト改善と勾配ノルムの小さい方に適用される。許容値を 0 に設定すると早期終了が無効になる。

.. _option-ls_iterations:

:at:`ls_iterations`: :at-val:`int, "50"`
   CG/Newton 制約ソルバーが実行する直線探索の最大反復回数。各制約解決中に最大で :ref:`iterations<option-iterations>` 回の :ref:`ls_iterations<option-ls_iterations>` 回の直線探索反復が実行されることを保証する。

.. _option-ls_tolerance:

:at:`ls_tolerance`: :at-val:`real, "0.01"`
   直線探索アルゴリズムの早期終了に使用される許容閾値。

.. _option-noslip_iterations:

:at:`noslip_iterations`: :at-val:`int, "0"`
   Noslip ソルバーの最大反復回数。これはメインソルバーの後に実行される後処理ステップである。修正PGS法を使用して、ソフト制約モデルに起因する摩擦次元での滑り/ドリフトを抑制する。デフォルト設定の 0 は、この後処理ステップを無効にする。

.. _option-noslip_tolerance:

:at:`noslip_tolerance`: :at-val:`real, "1e-6"`
   Noslip ソルバーの早期終了に使用される許容閾値。

.. _option-ccd_iterations:

:at:`ccd_iterations`: :at-val:`int, "50"`
   凸衝突に使用されるアルゴリズムの最大反復回数。通常、これを調整する必要はないが、一部のジオムが非常に大きいアスペクト比を持つ状況では調整が必要になる場合がある。

.. _option-ccd_tolerance:

:at:`ccd_tolerance`: :at-val:`real, "1e-6"`
   凸衝突アルゴリズムの早期終了に使用される許容閾値。

.. _option-sleep_tolerance:

:at:`sleep_tolerance`: :at-val:`real, "1e-4"`
   :ref:`sleeping<Sleeping>` が許可される速度許容値。

.. _option-sdf_iterations:

:at:`sdf_iterations`: :at-val:`int, "10"`
   符号付き距離場（SDF）衝突に使用される反復回数（初期点ごと）。

.. _option-sdf_initpoints:

:at:`sdf_initpoints`: :at-val:`int, "40"`
   符号付き距離場（SDF）衝突で接触を見つけるために使用される開始点の数。

.. youtube:: H9qG9Zf2W44
   :align: right
   :width: 240px

.. _option-actuatorgroupdisable:

:at:`actuatorgroupdisable`: :at-val:`int(31), optional`
   無効にするアクチュエータグループのリスト。 :ref:`group<actuator-general-group>` がこのリストに含まれるアクチュエータは力を生成しない。それらがステートフルな場合、その活性化状態は積分されない。内部的にこのリストは整数ビットフィールドとして実装されているため、値は ``0 <= group <= 30`` の範囲でなければならない。設定されていない場合、すべてのアクチュエータグループが有効になる。 `サンプルモデル <https://github.com/google-deepmind/mujoco/blob/main/test/engine/testdata/actuation/actuator_group_disable.xml>`__ と右側の画面キャプチャを参照。

.. _option-flag:

:el-prefix:`option/` |-| **flag** |?|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

この要素は、シミュレーションパイプラインの異なる部分を有効化または無効化するフラグを設定する。実行時に使用される実際のフラグは、mjModel.opt.disableflags と mjModel.opt.enableflags という2つの整数のビットとして表現される。これらはそれぞれ標準機能を無効にし、オプション機能を有効にするために使用される。この分離の理由は、両方の整数を 0 に設定するとデフォルトに戻るためである。XML内では、デフォルト属性値（標準機能に対応するフラグには "enable"、オプション機能に対応するフラグには "disable"）を除いて、この分離を明示的には行わない。以下のドキュメントでは、設定がデフォルトと異なる場合に何が起こるかを説明する。

.. _option-flag-constraint:

:at:`constraint`: :at-val:`[disable, enable], "enable"`
   このフラグは、制約ソルバーに関連するすべての標準計算を無効にする。その結果、制約力は適用されない。なお、次の4つのフラグは特定のタイプの制約に関連する計算を無効にする。この計算が実行されるには、このフラグと該当するタイプ固有のフラグの両方が "enable" に設定されている必要がある。

.. _option-flag-equality:

:at:`equality`: :at-val:`[disable, enable], "enable"`
   このフラグは、等式制約に関連するすべての標準計算を無効にする。

.. _option-flag-frictionloss:

:at:`frictionloss`: :at-val:`[disable, enable], "enable"`
   このフラグは、摩擦損失制約に関連するすべての標準計算を無効にする。

.. _option-flag-limit:

:at:`limit`: :at-val:`[disable, enable], "enable"`
   このフラグは、ジョイントとテンドンの制限制約に関連するすべての標準計算を無効にする。

.. _option-flag-contact:

:at:`contact`: :at-val:`[disable, enable], "enable"`
   このフラグは、衝突検出と接触制約に関連するすべての標準計算を無効にする。

.. _option-flag-spring:

:at:`spring`: :at-val:`[disable, enable], "enable"`
   このフラグは、受動的ジョイントとテンドンのバネを無効にする。受動的 :ref:`damper <option-flag-damper>` 力も無効にされている場合、**すべて** の受動的力が無効になる。これには重力補償、流体力、 :ref:`mjcb_passive` コールバックで計算される力、および :ref:`mjPLUGIN_PASSIVE<mjtPluginCapabilityBit>` ケイパビリティフラグが渡された際に :ref:`plugins <exPlugin>` で計算される力が含まれる。

.. _option-flag-damper:

:at:`damper`: :at-val:`[disable, enable], "enable"`
   このフラグは、受動的ジョイントとテンドンのダンパーを無効にする。受動的 :ref:`spring <option-flag-spring>` 力も無効にされている場合、**すべて** の受動的力が無効になる。これには重力補償、流体力、 :ref:`mjcb_passive` コールバックで計算される力、および :ref:`mjPLUGIN_PASSIVE<mjtPluginCapabilityBit>` ケイパビリティフラグが渡された際に :ref:`plugins <exPlugin>` で計算される力が含まれる。

.. _option-flag-gravity:

:at:`gravity`: :at-val:`[disable, enable], "enable"`
   このフラグは、mjOption 内の重力加速度ベクトルを実行時に (0 0 0) に置き換える。mjOption 内の値は変更されない。フラグが再び有効になると、mjOption 内の値が使用される。

.. _option-flag-clampctrl:

:at:`clampctrl`: :at-val:`[disable, enable], "enable"`
   このフラグは、アクチュエータ固有の属性がクランプを有効にするように設定されている場合でも、すべてのアクチュエータへの制御入力のクランプを無効にする。

.. _option-flag-warmstart:

:at:`warmstart`: :at-val:`[disable, enable], "enable"`
   このフラグは、制約ソルバーのウォームスタートを無効にする。デフォルトでは、ソルバーは前のタイムステップの解（すなわち制約力）を使用して反復最適化を初期化する。この機能は、軌道を形成しない状態のコレクションでダイナミクスを評価する場合には無効にすべきである。そのような場合、ウォームスタートは意味をなさず、ソルバーを遅くする可能性が高い。

.. _option-flag-filterparent:

:at:`filterparent`: :at-val:`[disable, enable], "enable"`
   このフラグは、2つのジオムが親ボディと子ボディに属する接触ペアのフィルタリングを無効にする。計算の章の接触 :ref:`selection <coSelection>` を参照。

.. _option-flag-actuation:

:at:`actuation`: :at-val:`[disable, enable], "enable"`
   このフラグは、アクチュエータダイナミクスを含む、アクチュエータ力に関連するすべての標準計算を無効にする。その結果、アクチュエータ力はシミュレーションに適用されない。

.. _option-flag-refsafe:

:at:`refsafe`: :at-val:`[disable, enable], "enable"`
   このフラグは、solref[0] がシミュレーションのタイムステップに比べて小さすぎることによる不安定性を防ぐ安全機構を有効にする。solref[0] は制約安定化に使用される仮想バネ・ダンパーの剛性であることを思い出してほしい。この設定が有効な場合、ソルバーはアクティブな各制約に対して個別に solref[0] の代わりに max(solref[0], 2*timestep) を使用する。

.. _option-flag-sensor:

:at:`sensor`: :at-val:`[disable, enable], "enable"`
   このフラグは、センサーに関連するすべての計算を無効にする。無効にされると、センサー値は一定のままになる。シミュレーション開始時に無効にされた場合はゼロ、実行時に無効にされた場合は最後に計算された値が保持される。

.. _option-flag-midphase:

:at:`midphase`: :at-val:`[disable, enable], "enable"`
   このフラグは、静的AABBバウンディングボリューム階層（BVHバイナリツリー）を使用したミッドフェーズ衝突フィルタリングを無効にする。無効にすると、衝突が許可されているすべてのジオムペアが衝突チェックされる。

.. _option-flag-nativeccd:

:at:`nativeccd`: :at-val:`[disable, enable], "enable"`
   このフラグは、 `libccd ライブラリ <https://github.com/danfis/libccd>`__ の代わりにネイティブ凸衝突検出パイプラインを有効にする。詳細は :ref:`convex collisions<coCCD>` を参照。

.. _option-flag-island:

:at:`island`: :at-val:`[disable, enable], "enable"`
   このフラグは、制約アイランドの発見と構築を有効にする。制約アイランドとは、相互作用しない制約と自由度の分離集合であり、独立に解くことができる。アイランディングはまだPGSソルバーではサポートされていない。詳細は :ref:`soIsland` を参照。 :ref:`mjVIS_ISLAND <mjtVisFlag>` は `アイランドの可視化 <https://youtu.be/Vc1tq0fFvQA>`__ を有効にする。

.. _option-flag-eulerdamp:

:at:`eulerdamp`: :at-val:`[disable, enable], "enable"`
   このフラグは、オイラー積分器におけるジョイント減衰に関する陰的積分を無効にする。詳細は :ref:`Numerical Integration<geIntegration>` セクションを参照。

.. _option-flag-autoreset:

:at:`autoreset`: :at-val:`[disable, enable], "enable"`
   このフラグは、数値的な問題が検出された際のシミュレーション状態の自動リセットを無効にする。

.. _option-flag-override:

:at:`override`: :at-val:`[disable, enable], "disable"`
   このフラグは :ref:`Contact override <COverride>` 機構を有効にする。

.. _option-flag-energy:

:at:`energy`: :at-val:`[disable, enable], "disable"`
   このフラグは、ポテンシャルエネルギーと運動エネルギーの計算を有効にする。これらはそれぞれ ``mjData.energy[0, 1]`` に格納され、simulate GUI の情報オーバーレイに表示される。ポテンシャルエネルギーには、すべてのボディにわたる重力成分の合計 :math:`\sum_b m_b g h` と、ジョイント、テンドン、フレックスの受動的バネに蓄えられるエネルギー :math:`\tfrac{1}{2} k x^2` が含まれる。ここで :math:`x` は変位、 :math:`k` はバネ定数である。運動エネルギーは :math:`\tfrac{1}{2} v^T M v` で与えられる。ここで :math:`v` は速度、 :math:`M` は質量行列である。なお、制約におけるポテンシャルエネルギーと運動エネルギーは考慮されていない。

   追加の計算（ :ref:`potential<sensor-e_potential>` および :ref:`kinetic<sensor-e_kinetic>` エネルギーセンサーによってもトリガーされる）はCPU時間を追加するが、通常は無視できる程度である。エネルギー保存されるはずのシステムのエネルギーを監視することは、複雑なシミュレーションの精度を評価する最良の方法の1つである。

.. _option-flag-fwdinv:

:at:`fwdinv`: :at-val:`[disable, enable], "disable"`
   このフラグは、順動力学と逆動力学の自動比較を有効にする。有効にすると、mj_forward（または mj_step 内部）の後に逆動力学が呼び出され、適用される力の差が mjData.solver_fwdinv[2] に記録される。最初の値は、ジョイント空間での不一致の相対ノルムで、次は制約空間である。

.. _option-flag-invdiscrete:

:at:`invdiscrete`: :at-val:`[disable, enable], "disable"`
   このフラグは、 ``RK4`` 以外のすべての :ref:`integrators<option-integrator>` に対して :ref:`mj_inverse` による離散時間逆動力学を有効にする。 :ref:`numerical integration<geIntegration>` セクションから、1ステップ積分器（ ``Euler``, ``implicit``, ``implicitfast`` ）は質量行列を :math:`M \rightarrow M-hD` と変更することを思い出してほしい。これは、有限差分加速度 :math:`(v_{t+h} - v_t)/h` が連続時間加速度 ``mjData.qacc`` と一致しないことを意味する。このフラグが有効な場合、 :ref:`mj_inverse` は ``qacc`` を2つの連続する速度の差から計算されたものと解釈し、上記の変更を元に戻す。


.. _option-flag-multiccd:

:at:`multiccd`: :at-val:`[disable, enable], "disable"`
   このフラグは、汎用凸‐凸コライダー（例：メッシュ‐メッシュ衝突）を使用するジオムペアに対して、マルチコンタクト衝突検出を有効にする。これは、接触するジオムが平面を持ち、凸‐凸コライダーが生成する単一の接触点では表面接触を正確に捉えられず、通常は滑りや揺れとして現れる不安定性につながる場合に有用である。この機能の実装は選択された凸衝突パイプラインに依存する。詳細は :ref:`convex collisions<coCCD>` を参照。

.. _option-flag-sleep:

:at:`sleep`: :at-val:`[disable, enable], "disable"`
   このフラグは :ref:`sleeping<Sleeping>` を有効にする。一部のツリーがスリープ中にこのフラグを無効にすると、それらをウェイクする。

   .. admonition:: flag value at initialization time
      :class: attention

      他のどの :ref:`flag<option-flag>` とも異なり、 :at:`sleep` フラグは :ref:`mjData` の **初期化** （ :ref:`mj_makeData` または :ref:`mj_resetData` ）中に効果を持つ。第一に、 :ref:`sleep-init<body-sleep>` ポリシーを有効にするには、初期化時に設定されている必要がある。第二に、静的量を計算するためには設定されている必要がある。詳細は :ref:`implementation notes<siSleep>` を参照。


.. _compiler:

**compiler** |m|
~~~~~~~~~~~~~~~~

この要素は、組み込みパーサーとコンパイラのオプションを設定するために使用される。解析とコンパイルの後は、もはや何の効果もない。ここでの設定はグローバルであり、モデル全体に適用される。

.. _compiler-autolimits:

:at:`autolimits`: :at-val:`[false, true], "true"`
   この属性は、"limited" （ <body-joint> または <tendon> 上）、"forcelimited"、"ctrllimited"、"actlimited" （ <actuator> 上）などの属性の動作に影響する。"true" の場合、これらの属性は不要であり、その値は対応する "range" 属性の存在から推測される。"false" の場合、そのような推測は行われない。ジョイントを制限するには、limited="true" と range="min max" の両方を指定する必要がある。このモードでは、制限なしで範囲を指定するとエラーになる。

.. _compiler-boundmass:

:at:`boundmass`: :at-val:`real, "0"`
   この属性は、ワールドボディを除く各ボディの質量に下限を課す。この属性を 0 より大きい値に設定すると、センサーを取り付けるためにURDFモデルでよく使用されるダミーボディなど、質量のない移動ボディを含む設計の悪いモデルに対する手っ取り早い修正として使用できる。MuJoCo ではダミーボディを作成する必要がないことに注意。

.. _compiler-boundinertia:

:at:`boundinertia`: :at-val:`real, "0"`
   この属性は、ワールドボディを除く各ボディの対角慣性成分に下限を課す。使用方法は上記の boundmass と同様である。

.. _compiler-settotalmass:

:at:`settotalmass`: :at-val:`real, "-1"`
   この値が正の場合、コンパイラはモデル内のすべてのボディの質量と慣性をスケーリングし、総質量がここで指定された値と等しくなるようにする。ワールドボディの質量は 0 で、質量関連の計算には関与しない。このスケーリングは、ボディの質量と慣性に影響するすべての他の操作の後、最後に実行される。同じスケーリング操作は、関数 :ref:`mj_setTotalmass` を使用して、コンパイル済みの mjModel に対して実行時に適用できる。

.. _compiler-balanceinertia:

:at:`balanceinertia`: :at-val:`[false, true], "false"`
   有効な対角慣性行列は、3つの対角要素のすべての順列に対して A+B>=C を満たす必要がある。設計の悪い一部のモデルはこの制約に違反し、通常はコンパイルエラーになる。この属性が "true" に設定されている場合、上記の条件に違反するたびに、コンパイラは3つの対角要素すべてをそれらの平均値に静かに設定する。

.. _compiler-strippath:

:at:`strippath`: :at-val:`[false, true], "false" for MJCF, "true" for URDF`
   この属性が "true" の場合、パーサーはモデルで指定されたファイル名からパス情報を削除する。これは、異なるディレクトリ構造を使用する別のシステムで作成されたモデルをロードする際に有用である。

.. _compiler-coordinate:

:at:`coordinate`: :at-val:`[local, global], "local"`
   以前のバージョンでは、この属性はフレームの位置と向きをローカル座標とグローバル座標のどちらで表現するかを指定するために使用できたが、"global" オプションはその後削除され、エラーを生成するようになった。"global" オプションを使用していた古いモデルを変換するには、MuJoCo 2.3.3 以前でロードして保存する必要がある。

.. _compiler-angle:

:at:`angle`: :at-val:`[radian, degree], "degree" for MJCF, always "radian" for URDF`
   この属性は、MJCF モデルの角度が度またはラジアンのどちらの単位で表現されているかを指定する。コンパイラは度をラジアンに変換し、mjModel は常にラジアンを使用する。URDF モデルの場合、パーサーは XML 設定に関係なく、この属性を内部的に "radian" に設定する。

.. image:: images/changelog/meshfit.png
   :align: right
   :width: 40%

.. _compiler-fitaabb:

:at:`fitaabb`: :at-val:`[false, true], "false"`
   コンパイラは、メッシュをそのメッシュにフィットした幾何プリミティブに置き換えることができる。以下の :ref:`geom <body-geom>` を参照。この属性が "true" の場合、フィッティング手順はメッシュの軸並行バウンディングボックス（AABB）を使用し、AABB がメッシュの AABB を含む最小のプリミティブを選択する。それ以外の場合は、メッシュの等価慣性ボックスを使用する。フィッティングに使用される幾何プリミティブのタイプは、各ジオムに対して個別に指定される。右側の画像の生成に使用されたモデルは `こちら <https://github.com/google-deepmind/mujoco/blob/main/test/user/testdata/fitmesh_inertiabox.xml>`__ （fit inertia box）と `こちら <https://github.com/google-deepmind/mujoco/blob/main/test/user/testdata/fitmesh_aabb.xml>`__ （fit aabb）にある。

.. _compiler-eulerseq:

:at:`eulerseq`: :at-val:`string, "xyz"`
   この属性は、空間フレームを持つ要素のすべての :at:`euler` 属性に対するオイラー回転のシーケンスを指定する。 :ref:`COrientation` で説明されているとおり。これは {x, y, z, X, Y, Z} の集合から正確に3文字の文字列でなければならない。位置 n の文字は、n 番目の回転が実行される軸を決定する。小文字はフレームとともに回転する軸（内的）を示し、大文字は親フレーム内で固定されたままの軸（外的）を示す。URDF で使用される "rpy" 規約は、MJCF では "XYZ" に対応する。

.. _compiler-meshdir:

:at:`meshdir`: :at-val:`string, optional`
   この属性は、メッシュとハイトフィールドファイルを検索する場所をコンパイラに指示する。ファイルへのフルパスは次のように決定される。上記の strippath 属性が "true" の場合、ファイル名からすべてのパス情報が削除される。次に、次のチェックが順番に適用される。(1) ファイル名に絶対パスが含まれている場合、それ以上の変更なしに使用される。(2) この属性が設定され、絶対パスが含まれている場合、フルパスはここで指定された文字列にファイル名を追加したものになる。(3) フルパスは、メイン MJCF モデルファイルへのパスに、指定されている場合はこの属性の値を追加し、ファイル名を追加したものになる。

.. _compiler-texturedir:

:at:`texturedir`: :at-val:`string, optional`
   この属性は、テクスチャファイルを検索する場所をコンパイラに指示するために使用される。上記の meshdir と同じ方法で機能する。

.. _compiler-assetdir:

:at:`assetdir`: :at-val:`string, optional`
   この属性は、上記の :at:`meshdir` と :at:`texturedir` の両方の値を設定する。後者の属性の値は :at:`assetdir` よりも優先される。

.. _compiler-discardvisual:

:at:`discardvisual`: :at-val:`[false, true], "false" for MJCF, "true" for URDF`
   この属性は、純粋に視覚的で物理に影響を与えないすべてのモデル要素を破棄するようコンパイラに指示する（1つの例外を除く、以下を参照）。これにより、 :ref:`mjModel` 構造体のサイズを小さくし、シミュレーションを高速化できることがよくある。

   - すべてのマテリアルが破棄される。
   - すべてのテクスチャが破棄される。
   - :ref:`contype<body-geom-contype>` |-| = |-| :ref:`conaffinity<body-geom-conaffinity>` |-| =0 のすべてのジオムは、他の MJCF 要素で参照されていない場合、破棄される。破棄されたジオムがボディ慣性の推測に使用されていた場合、明示的な :ref:`inertial<body-inertial>` 要素がボディに追加される。
   - どのジオムからも参照されていないすべてのメッシュ（特に上記で破棄されたもの）が破棄される。

   結果としてコンパイルされたモデルは、元のモデルとまったく同じダイナミクスを持つ。変更される可能性のある唯一のエンジンレベルの計算は、 :ref:`rangefinder<sensor-rangefinder>` センサーなどで使用される :ref:`raycasting<mj_ray>` 計算の出力である。これは、レイキャストが視覚ジオムまでの距離を報告するためである。このフラグでコンパイルされたモデルを可視化する際には、衝突ジオムがデフォルトで非表示になっている :ref:`group<body-geom-group>` に配置されていることが多いことを覚えておくことが重要である。

.. _compiler-usethread:

:at:`usethread`: :at-val:`[false, true], "true"`
   この属性が "true" の場合、モデルコンパイラはマルチスレッドモードで実行される。現在、マルチスレッドはアクチュエータの長さ範囲の計算と、メッシュの並列読み込みおよび処理に使用されている。

.. _compiler-fusestatic:

:at:`fusestatic`: :at-val:`[false, true], "false" for MJCF, "true" for URDF`
   この属性は、静的ボディがその親と融合され、それらのボディで定義された要素が親に再割り当てされるコンパイラ最適化機能を制御する。静的ボディは、次の場合を除いて親と融合される。

   - モデル内の別の要素から参照されている。
   - :ref:`force<sensor-force>` または :ref:`torque<sensor-torque>` センサーから参照されているサイトが含まれている。

   この最適化は、多くのダミーボディを持つことが多い URDF モデルをインポートする際に特に有用であるが、MJCF モデルの最適化にも使用できる。最適化後、新しいモデルは元のモデルと同一の運動学とダイナミクスを持つが、シミュレーションが高速である。

.. _compiler-inertiafromgeom:

:at:`inertiafromgeom`: :at-val:`[false, true, auto], "auto"`
   この属性は、ボディに取り付けられたジオムからボディの質量と慣性を自動推測することを制御する。この設定が "false" の場合、自動推測は実行されない。その場合、各ボディは :ref:`inertial <body-inertial>` 要素で明示的に定義された質量と慣性を持つ必要があり、そうでない場合はコンパイルエラーが生成される。この設定が "true" の場合、各ボディの質量と慣性はそれに取り付けられたジオムから推測され、 :el:`inertial` 要素で指定された値を上書きする。デフォルト設定 "auto" は、ボディ定義で :el:`inertial` 要素が欠けている場合にのみ、質量と慣性が自動的に推測されることを意味する。この属性を "auto" ではなく "true" に設定する理由の1つは、設計の悪いモデルからインポートされた慣性データを上書きすることである。特に、公開されている多くの URDF モデルは、質量に比べて大きすぎる任意の慣性を持っているように見える。これにより、モデルの幾何学的境界をはるかに超えて拡張する等価慣性ボックスが生成される。組み込みの OpenGL ビジュアライザーは等価慣性ボックスをレンダリングできることに注意。

.. _compiler-alignfree:

:at:`alignfree`: :at-val:`[false, true], "false"`
   この属性は、 :ref:`free joint<body-freejoint>` を持ち、子ボディを持たないボディに適用される最適化のデフォルト動作を切り替える。true の場合、ボディフレームとフリージョイントが慣性フレームと自動的に整列され、より高速で安定したシミュレーションが実現される。詳細は :ref:`freejoint/align<body-freejoint-align>` を参照。

.. _compiler-inertiagrouprange:

:at:`inertiagrouprange`: :at-val:`int(2), "0 5"`
   この属性は、ボディの質量と慣性の推測に使用されるジオムグループの範囲を指定する（そのような推測が有効な場合）。 :ref:`geom <body-geom>` の group 属性は整数である。この整数がここで指定された範囲に含まれる場合、ジオムは慣性計算に使用され、そうでない場合は無視される。この機能は、衝突と可視化のための冗長なジオムセットを持つモデルで有用である。ワールドボディは慣性計算に関与しないため、それに取り付けられたジオムは自動的に無視される。したがって、ワールドジオムを慣性計算から除外するために、この属性とジオム固有のグループを調整する必要はない。

.. _compiler-saveinertial:

:at:`saveinertial`: :at-val:`[false, true], "false"`
   "true" に設定すると、コンパイラはすべてのボディに対して明示的な :ref:`inertial <body-inertial>` 句を保存する。

.. _compiler-lengthrange:

:el-prefix:`compiler/` |-| **lengthrange** |?|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

この要素は、アクチュエータ長さ範囲の計算を制御する。この機能の概要については :ref:`Length range <CLengthRange>` セクションを参照。この要素が省略された場合でも、以下に示すデフォルトが適用される。長さ範囲の計算を完全に無効にするには、この要素を含めて mode="none" に設定する。

.. _compiler-lengthrange-mode:

:at:`mode`: :at-val:`[none, muscle, muscleuser, all], "muscle"`
   長さ範囲計算が適用されるアクチュエータのタイプを決定する。"none" はこの機能を無効にする。"all" はすべてのアクチュエータに適用する。"muscle" は gaintype または biastype が "muscle" に設定されているアクチュエータに適用する。"muscleuser" は gaintype または biastype が "muscle" または "user" のいずれかに設定されているアクチュエータに適用する。デフォルトは "muscle" である。これは MuJoCo の筋肉モデルがアクチュエータ長さ範囲を定義する必要があるためである。

.. _compiler-lengthrange-useexisting:

:at:`useexisting`: :at-val:`[false, true], "true"`
   この属性が "true" で、指定されたアクチュエータの長さ範囲がモデルで既に定義されている場合、既存の値が使用され、自動計算はスキップされる。範囲は、最初の数値が2番目の数値よりも小さい場合に定義されていると見なされる。この属性を "false" に設定する唯一の理由は、アクチュエータ長さ範囲の再計算を強制することである。これは、モデルのジオメトリが変更された場合に必要になる。自動計算はシミュレーションに依存し、遅くなる可能性があるため、モデルを保存し、可能な場合は既存の値を使用することが推奨される。

.. _compiler-lengthrange-uselimit:

:at:`uselimit`: :at-val:`[false, true], "false"`
   この属性が "true" で、アクチュエータが制限が定義されたジョイントまたはテンドンに取り付けられている場合、これらの制限がアクチュエータ長さ範囲にコピーされ、自動計算はスキップされる。これは良いアイデアのように思えるかもしれないが、複雑なモデルでは、テンドンアクチュエータの実現可能な範囲はモデル全体に依存し、そのテンドンのユーザー定義制限よりも小さい場合があることに注意。したがって、より安全なアプローチは、これを "false" に設定し、自動計算に実現可能な範囲を発見させることである。

.. _compiler-lengthrange-accel:

:at:`accel`: :at-val:`real, "20"`
   この属性は、各アクチュエータを最小および最大長さに押し込むために、シミュレーションに適用される力をスケーリングする。力の大きさは、結果として生じる関節空間加速度ベクトルのノルムがこの属性と等しくなるように計算される。

.. _compiler-lengthrange-maxforce:

:at:`maxforce`: :at-val:`real, "0"`
   上記の accel 属性を介して計算される力は、アクチュエータが非常に小さいモーメントを持つ場合、非常に大きくなる可能性がある。そのような力は依然として（構造上）妥当な加速度を生成するが、大きな数値は数値的な問題を引き起こす可能性がある。そのような問題は観察されたことはないが、現在の属性は安全策として提供されている。0 より大きい値に設定すると、シミュレーション中に適用される力のノルムが制限される。デフォルト設定の 0 は、この安全策を無効にする。

.. _compiler-lengthrange-timeconst:

:at:`timeconst`: :at-val:`real, "1"`
   シミュレーションは、不安定性のリスクなしにアクチュエータを制限に押し込むために、非物理的な方法で減衰される。これは、各タイムステップで関節速度を単純にスケールダウンすることによって行われる。新しい加速度がない場合、そのようなスケーリングは速度を指数関数的に減少させる。timeconst 属性は、この指数関数的減少の時定数を秒単位で指定する。

.. _compiler-lengthrange-timestep:

:at:`timestep`: :at-val:`real, "0.01"`
   内部シミュレーションに使用されるタイムステップ。これを 0 に設定すると、モデルのタイムステップが使用される。後者がデフォルトでないのは、不安定になる可能性のあるモデルは通常小さなタイムステップを持っているが、ここでのシミュレーションは人工的に減衰されており非常に安定しているためである。長さ範囲計算を高速化するために、ユーザーはこの値を増やすことを試みることができる。

.. _compiler-lengthrange-inttotal:

:at:`inttotal`: :at-val:`real, "10"`
   各アクチュエータとアクチュエータ方向に対して、内部シミュレーションを実行するための合計時間間隔（秒単位）。各シミュレーションは qpos0 で初期化される。inttotal 時間が経過した後に落ち着くことが期待される。

.. _compiler-lengthrange-interval:

:at:`interval`: :at-val:`real, "2"`
   シミュレーションの終わりの時間間隔。この間隔で長さデータが収集され、分析される。この間隔中に達成された最大（またはそれぞれ最小）長さが記録される。最大と最小の差も記録され、発散の尺度として使用される。シミュレーションが落ち着いた場合、この差は小さくなる。小さくない場合、これはシミュレーションがまだ落ち着いていないため（その場合、上記の属性を調整する必要がある）か、モデルに十分なジョイントとテンドンの制限がなく、アクチュエータ範囲が事実上無制限であるためである可能性がある。これらの条件は両方とも同じコンパイラエラーを引き起こす。このシミュレーションでは接触が無効になっているため、ジョイントとテンドンの制限、および全体的なジオメトリだけがアクチュエータの無限長を防ぐことができることを思い出してほしい。

.. _compiler-lengthrange-tolrange:

:at:`tolrange`: :at-val:`real, "0.05"`
   これは、発散を検出してコンパイラエラーを生成するための閾値を決定する。interval 中に観察されたアクチュエータ長さの範囲が、シミュレーションを介して計算された全体的な範囲で除算される。その値が tolrange よりも大きい場合、コンパイラエラーが生成される。したがって、コンパイラエラーを抑制する1つの方法は、この属性を単に大きくすることであるが、その場合、結果が不正確になる可能性がある。


.. _size:

**size** |m|
~~~~~~~~~~~~

この要素は、モデル内の要素数から推測できないサイズパラメータを指定する。実行時に変更できる mjOption のフィールドとは異なり、サイズは構造パラメータであり、コンパイル後に変更すべきではない。

.. _size-memory:

:at:`memory`: :at-val:`string, "-1"`
   この属性は、 ``mjData.arena`` メモリ空間内の動的配列に割り当てられるメモリのサイズをバイト単位で指定する。デフォルト設定の ``-1`` は、コンパイラに割り当てる空間の大きさを推測するよう指示する。数字の後に {K, M, G, T, P, E} のいずれかの文字を追加すると、単位がそれぞれ {キロ、メガ、ギガ、テラ、ペタ、エクサ}バイトに設定される。したがって、"16M" は "16メガバイトの ``arena`` メモリを割り当てる" ことを意味する。詳細は :ref:`Memory allocation <CSize>` セクションを参照。

.. _size-njmax:

:at:`njmax`: :at-val:`int, "-1"` |nbsp| |nbsp| |nbsp| (legacy)
   これは非推奨のレガシー属性である。バージョン 2.3.0 より前では、許可される制約の最大数を決定していた。現在は「この数の制約に以前必要だったのと同じ量のメモリを割り当てる」ことを意味する。 :at:`njmax` と :at:`memory` の両方を指定するとエラーになる。

.. _size-nconmax:

:at:`nconmax`: :at-val:`int, "-1"` |nbsp| |nbsp| |nbsp| (legacy)
   この属性は、実行時に生成される接触の最大数を指定する。アクティブな接触の数がこの値を超えようとする場合、余分な接触は破棄され、警告が生成される。これは、バージョン 2.3.0 より前にメモリ割り当てに影響した非推奨のレガシー属性である。後方互換性とデバッグ目的のために保持されている。

.. _size-nstack:

:at:`nstack`: :at-val:`int, "-1"` |nbsp| |nbsp| |nbsp| (legacy)
   これは非推奨のレガシー属性である。バージョン 2.3.0 より前では、 :ref:`stack <siStack>` の最大サイズを決定していた。バージョン 2.3.0 以降、 :at:`nstack` が指定されている場合、 ``mjData.narena`` のサイズは ``nstack * sizeof(mjtNum)`` バイトに、制約ソルバーのための追加スペースを加えたものになる。 :at:`nstack` と :at:`memory` の両方を指定するとエラーになる。

.. _size-nuserdata:

:at:`nuserdata`: :at-val:`int, "0"`
   mjData の mjData.userdata フィールドのサイズ。このフィールドはカスタム動的変数を保存するために使用すべきである。 :ref:`CUser` も参照。

.. _size-nkey:

:at:`nkey`: :at-val:`int, "0"`
   mjModel に割り当てられるキーフレームの数は、この値と以下の :ref:`key <keyframe-key>` 要素の数の大きい方である。インタラクティブシミュレーターには、システム状態のスナップショットを撮り、キーフレームとして保存する機能があることに注意。

.. _size-nuser_body:

:at:`nuser_body`: :at-val:`int, "-1"`
   各ボディの定義に追加されるカスタムユーザーパラメータの数。 :ref:`User parameters <CUser>` も参照。パラメータ値は :ref:`body <body>` 要素の user 属性を介して設定される。これらの値は MuJoCo によってアクセスされない。ユーザーコールバックやその他のカスタムコードで必要な要素プロパティを定義するために使用できる。

.. _size-nuser_jnt:

:at:`nuser_jnt`: :at-val:`int, "-1"`
   各 :ref:`joint <body-joint>` の定義に追加されるカスタムユーザーパラメータの数。

.. _size-nuser_geom:

:at:`nuser_geom`: :at-val:`int, "-1"`
   各 :ref:`geom <body-geom>` の定義に追加されるカスタムユーザーパラメータの数。

.. _size-nuser_site:

:at:`nuser_site`: :at-val:`int, "-1"`
   各 :ref:`site <body-site>` の定義に追加されるカスタムユーザーパラメータの数。

.. _size-nuser_cam:

:at:`nuser_cam`: :at-val:`int, "-1"`
   各 :ref:`camera <body-camera>` の定義に追加されるカスタムユーザーパラメータの数。

.. _size-nuser_tendon:

:at:`nuser_tendon`: :at-val:`int, "-1"`
   各 :ref:`tendon <tendon>` の定義に追加されるカスタムユーザーパラメータの数。

.. _size-nuser_actuator:

:at:`nuser_actuator`: :at-val:`int, "-1"`
   各 :ref:`actuator <actuator>` の定義に追加されるカスタムユーザーパラメータの数。

.. _size-nuser_sensor:

:at:`nuser_sensor`: :at-val:`int, "-1"`
   各 :ref:`sensor <sensor>` の定義に追加されるカスタムユーザーパラメータの数。


.. _statistic:

**statistic** |m|
~~~~~~~~~~~~~~~~~

この要素は、コンパイラによって計算されたモデル統計をオーバーライドするために使用される。これらの統計は情報提供だけでなく、レンダリングと摂動のさまざまなコンポーネントをスケーリングするためにも使用される。XML でオーバーライド機構を提供するのは、多数の視覚パラメータを調整するよりも、少数のモデル統計を調整する方が簡単な場合があるためである。

.. _statistic-meanmass:

:at:`meanmass`: :at-val:`real, optional`
   この属性が指定されている場合、コンパイラによって計算された mjModel.stat.meanmass の値を置き換える。計算される値は、質量のないワールドボディを除く平均ボディ質量である。実行時に、この値は摂動力をスケーリングする。

.. _statistic-meaninertia:

:at:`meaninertia`: :at-val:`real, optional`
   この属性が指定されている場合、コンパイラによって計算された mjModel.stat.meaninertia の値を置き換える。計算される値は、モデルが qpos0 にあるときの関節空間慣性行列の平均対角要素である。実行時に、この値は早期終了に使用されるソルバーコストと勾配をスケーリングする。

.. _statistic-meansize:

:at:`meansize`: :at-val:`real, optional`
   この属性が指定されている場合、コンパイラによって計算された ``mjModel.stat.meansize`` の値を置き換える。実行時に、この値は上記の :ref:`scale <visual-scale>` 要素の属性を乗算し、それらの長さ単位として機能する。特定の長さが望ましい場合、 :at:`meansize` を 1 や 0.01 などの丸い数値に設定すると便利である。これにより、 :ref:`scale <visual-scale>` 値が認識された長さ単位になる。これが :at:`meansize` の唯一の意味であり、これを設定しても他の副作用はない。自動計算される値はヒューリスティックであり、平均ボディ半径を表す。ヒューリスティックは、存在する場合はジオムサイズ、存在する場合はジョイント間の距離、およびボディ等価慣性ボックスのサイズに基づいている。

.. _statistic-extent:

:at:`extent`: :at-val:`real, optional`
   この属性が指定されている場合、コンパイラによって計算された mjModel.stat.extent の値を置き換える。計算される値は、初期構成でのモデルのバウンディングボックスの辺の半分である。実行時に、この値は上記の :ref:`map <visual-map>` 要素の一部の属性と乗算される。モデルが最初にロードされたとき、フリーカメラの :at:`center` （以下を参照）からの初期距離は :at:`extent` の 1.5 倍である。厳密に正である必要がある。

.. _statistic-center:

:at:`center`: :at-val:`real(3), optional`
   この属性が指定されている場合、コンパイラによって計算された mjModel.stat.center の値を置き換える。計算される値は、初期構成でのモデル全体のバウンディングボックスの中心である。この 3D ベクトルは、モデルが最初にロードされたときにフリーカメラのビューを中心に配置するために使用される。



.. _asset:

**asset** |m|
~~~~~~~~~~~~~

これはアセットを定義するためのグルーピング要素です。属性はありません。アセットはモデル内で作成され、他のモデル要素から参照できるようにします。概要の章の :ref:`Assets <Assets>` の説明を参照してください。ファイルから開かれたアセットは、ファイル名拡張子または ``content_type`` 属性の2つの方法で識別できます。MuJoCoは指定されたコンテンツタイプによってファイルを開こうとし、 ``content_type`` 属性が指定されていない場合のみファイル名拡張子にフォールバックします。アセットがファイルから読み込まれない場合、コンテンツタイプは無視されます。


.. _asset-mesh:

:el-prefix:`asset/` |-| **mesh** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

この要素はメッシュアセットを作成し、ジオムから参照できるようになります。参照するジオムのタイプが :at-val:`mesh` の場合、メッシュはモデル内でインスタンス化されます。それ以外の場合、ジオメトリックプリミティブが自動的にフィットされます。以下の :ref:`geom <body-geom>` 要素を参照してください。

MuJoCoは三角形分割されたメッシュを扱います。メッシュはバイナリSTLファイル、OBJファイル、または以下で説明するカスタム形式のMSHファイルから読み込むことができます。また、XMLで頂点と面のデータを直接指定することもできます。MeshLabなどのソフトウェアを使用して、他のメッシュ形式からSTLまたはOBJに変換できます。任意の三角形の集合をメッシュとして読み込んでレンダリングできますが、衝突検出は :ref:`Collision` で説明されているようにメッシュの凸包を使用します。メッシュの外観（テクスチャマッピングを含む）は、参照するジオムの :at:`material` と :at:`rgba` 属性によって制御されます。これはハイトフィールドと同様です。

メッシュは、自動テクスチャマッピング機構に依存する代わりに、明示的なテクスチャ座標を持つことができます。提供された場合、これらの明示的な座標が優先されます。テクスチャ座標は、OBJファイルとMSHファイルで指定でき、また :at:`texcoord` 属性でXMLに明示的に指定できますが、STLファイルでは指定できません。これらの機構を混在させることはできません。したがって、STLメッシュにテクスチャ座標を追加する唯一の方法は、サポートされている他の形式のいずれかに変換することです。

.. _legacy-msh-docs:

.. collapse:: レガシーMSHファイル形式

   バイナリMSHファイルは、頂点位置の数（nvertex）、頂点法線の数（nnormal）、頂点テクスチャ座標の数（ntexcoord）、面を構成する頂点インデックスの数（nface）を指定する4つの整数で始まり、その後に数値データが続きます。nvertexは少なくとも4でなければなりません。nnormalとntexcoordは0（その場合、対応するデータは定義されません）またはnvertexと等しくなければなりません。nfaceも0にできます。その場合、面は頂点位置の凸包から自動的に構築されます。ファイルサイズ（バイト単位）は正確に 16 + 12*(nvertex + nnormal + nface) + 8*ntexcoord でなければなりません。ファイルの内容は以下の通りです:

   .. code:: Text

          (int32)   nvertex
          (int32)   nnormal
          (int32)   ntexcoord
          (int32)   nface
          (float)   vertex_positions[3*nvertex]
          (float)   vertex_normals[3*nnormal]
          (float)   vertex_texcoords[2*ntexcoord]
          (int32)   face_vertex_indices[3*nface]

設計の悪いメッシュは、レンダリングアーティファクトを表示する可能性があります。特に、シャドウマッピング機構は、前向きと後向きの三角形面の間にある程度の距離があることに依存しています。面が繰り返され、各三角形の頂点順序によって決定される法線が逆向きである場合、シャドウエイリアシングが発生します。解決策は、繰り返された面を削除するか（MeshLabで実行できます）、より適切に設計されたメッシュを使用することです。反転した面は、OBJまたはXMLとして指定されたメッシュに対してMuJoCoによってチェックされ、エラーメッセージが返されます。

メッシュのサイズは、メッシュファイル内の頂点データの3D座標に、以下の :at:`scale` 属性の成分を乗じたものによって決定されます。スケーリングは各座標軸に対して個別に適用されます。負のスケーリング値を使用してメッシュを反転できることに注意してください。これは正当な操作です。参照するジオムのサイズパラメータは、ハイトフィールドと同様に無視されます。また、 :ref:`refpos<asset-mesh-refpos>` と :ref:`refquat<asset-mesh-refquat>` 属性を使用して、3D座標を平行移動および回転する機構も提供しています。

メッシュは面なしでも定義できます（基本的にはポイントクラウド）。その場合、凸包は自動的に構築されます。これにより、XMLで直接単純な形状を構築することが簡単になります。たとえば、ピラミッドは次のように作成できます:

.. code-block:: xml

   <asset>
     <mesh name="tetrahedron" vertex="0 0 0  1 0 0  0 1 0  0 0 1"/>
   </asset>

位置決めと方向付けは、ソースアセット内の頂点データが、原点がメッシュ内にない座標フレームに対して相対的であることが多いという事実により複雑になります。対照的に、MuJoCoはジオムのローカルフレームの原点が形状の幾何学的中心と一致することを期待しています。この不一致を解決するために、コンパイラでメッシュを前処理し、(0,0,0)を中心とし、その主慣性軸が座標軸になるようにします。ソースアセットに適用された平行移動と回転のオフセットを :ref:`mjModel.mesh_pos<mjModel>` と :ref:`mjModel.mesh_quat<mjModel>` に保存します。これらは、ソースから頂点データを読み取り、変換を再適用する必要がある場合に必要です。これらのオフセットは、参照するジオムの位置と向きと組み合わされます。以下の :ref:`geom <body-geom>` の :at:`mesh` 属性も参照してください。幸いなことに、ロボットモデルで使用されるほとんどのメッシュは、ジョイントを中心とした座標フレームで設計されています。これにより、対応するMJCFモデルが直感的になります。ジョイント位置がボディフレーム内で(0,0,0)になるようにボディフレームをジョイントに設定し、単にメッシュを参照します。以下は前腕のMJCFモデルフラグメントで、メッシュを期待される場所に配置するために必要なすべての情報を含んでいます。ボディの位置は親ボディ、すなわち上腕（図示されていません）に対して相対的に指定されています。これは35 cmでオフセットされており、これは人間の上腕の典型的な長さです。メッシュの頂点データが上記の規約で設計されていない場合、ジオムの位置と向き（または :at:`refpos`, :at:`refquat` 機構）を使用して補正する必要がありますが、実際にはこれが必要になることはほとんどありません。

.. code-block:: xml

   <asset>
     <mesh file="forearm.stl"/>
   </asset>

   <body pos="0 0 0.35"/>
     <joint type="hinge" axis="1 0 0"/>
     <geom type="mesh" mesh="forearm"/>
   </body>

上記の慣性計算は、メッシュを中心に配置して整列させるだけでなく、それが取り付けられているボディの質量と慣性を推測するために使用されるアルゴリズムの一部です。これは、三角形面の重心を計算し、各面を重心と接続して三角錐を形成し、すべての錐の質量と符号付き慣性を計算（ソリッドと見なす、または :at:`shellinertia` がtrueの場合は中空）し、それらを累積することによって行われます。符号により、表面の外側にある錐が減算されることが保証されます。これは凹形状で発生する可能性があります。このアルゴリズムは、Joseph O'RourkeのComputational Geometry in C (Second Edition)のセクション1.3.8にあります。

コンパイラが各メッシュに適用する処理ステップの完全なリストは次のとおりです:

#. STLメッシュの場合、繰り返された頂点を削除し、必要に応じて面を再インデックスします。メッシュがSTLでない場合、必要な頂点と面がすでに生成されていると想定し、削除または再インデックスは適用しません;
#. 頂点法線が提供されていない場合、周囲の面法線の加重平均を使用して法線を自動的に生成します。鋭いエッジが検出された場合、レンダラーは :ref:`smoothnormal<asset-mesh-smoothnormal>` がtrueでない限り、面法線を使用してエッジに関する視覚情報を保持します。STLメッシュでは法線を提供できないことに注意してください;
#. 頂点と法線をスケーリング、平行移動、回転し、スケーリングの場合は法線を再正規化します。これらの変換を ``mjModel.mesh_{pos, quat, scale}`` に保存します。
#. 指定されている場合、凸包を構築します;
#. すべての三角形面の重心を見つけ、錐の和表現を構築します。面積が小さすぎる（ :ref:`mjMINVAL <glNumeric>` 値の1E-14未満）三角形は、コンパイルエラーになります;
#. 錐の和の質量中心と慣性行列を計算します。固有値分解を使用して主慣性軸を見つけます。メッシュを中心に配置して整列し、後続のジオム関連の計算のために平行移動と回転のオフセットを保存します。

.. _asset-mesh-name:

:at:`name`: :at-val:`string, optional`
   メッシュの名前。参照に使用されます。省略された場合、メッシュ名はパスと拡張子を除いたファイル名と同じになります。

.. _asset-mesh-class:

:at:`class`: :at-val:`string, optional`
   未指定の属性（この場合はscaleのみ）を設定するためのデフォルトクラス。

.. _asset-mesh-content_type:

:at:`content_type`: :at-val:`string, optional`
   file属性が指定されている場合、これは読み込まれるファイルの `Media Type <https://www.iana.org/assignments/media-types/media-types.xhtml>`__ （以前はMIMEタイプとして知られていました）を設定します。ファイル名拡張子はオーバーロードされます。現在、 ``model/vnd.mujoco.msh``、 ``model/obj``、および ``model/stl`` がサポートされています。

.. _asset-mesh-file:

:at:`file`: :at-val:`string, optional`
   メッシュが読み込まれるファイル。パスは :ref:`compiler <compiler>` のmeshdir属性で説明されているように決定されます。ファイル拡張子は、ファイルタイプを指定する "stl"、"msh"、または "obj"（大文字と小文字を区別しない）でなければなりません。ファイル名が省略された場合、vertex属性が必須になります。

.. _asset-mesh-scale:

:at:`scale`: :at-val:`real(3), "1 1 1"`
   この属性は、各座標軸に沿って頂点データに適用されるスケーリングを指定します。負の値が許可され、対応する軸に沿ってメッシュを反転させます。

.. _asset-mesh-inertia:

:at:`inertia`: :at-val:`[convex, exact, legacy, shell], "legacy"`
   この属性は、質量と慣性が :ref:`形状から推測される<compiler-inertiafromgeom>` ときにメッシュがどのように使用されるかを制御します。現在のデフォルト値 :at-val:`legacy` は、将来のリリースで :at-val:`convex` に変更されます。

   :at-val:`convex`: メッシュの凸包を使用して、一様密度を仮定して体積と慣性を計算します。

   :at-val:`exact`: 非凸メッシュでも体積と慣性を正確に計算します。このアルゴリズムは、適切に方向付けされた水密メッシュを必要とし、そうでない場合はエラーになります。

   :at-val:`legacy`: レガシーアルゴリズムを使用します。非凸メッシュで体積の過剰カウントにつながります。現在は破壊を避けるためにデフォルトですが、推奨されません。

   :at-val:`shell`: 質量がメッシュの表面に集中していると仮定します。メッシュの表面を使用して、一様表面密度を仮定して慣性を計算します。

.. _asset-mesh-smoothnormal:

:at:`smoothnormal`: :at-val:`[false, true], "false"`
   法線が明示的に与えられていない場合の頂点法線の自動生成を制御します。trueの場合、面積に比例した重みで各頂点の面法線を平均化することにより、滑らかな法線が生成されます。falseの場合、平均法線に対して大きな角度の面は平均から除外されます。このようにして、鋭いエッジ（立方体のエッジなど）は滑らかにされません。

.. _asset-mesh-maxhullvert:

:at:`maxhullvert`: :at-val:`int, "-1"`
   メッシュの凸包の最大頂点数。現在、これはqhullに :at:`maxhullvert` 頂点後に `終了するように要求する <http://www.qhull.org/html/qh-optt.htm#TAn>`__ ことによって実装されています。デフォルト値の-1は「無制限」を意味します。正の値は3より大きくなければなりません。

.. _asset-mesh-vertex:

:at:`vertex`: :at-val:`real(3*nvert), optional`
   頂点の3D位置データ。XMLでこの属性を使用して位置データを指定するか、バイナリファイルを使用できますが、両方を使用することはできません。

.. _asset-mesh-normal:

:at:`normal`: :at-val:`real(3*nvert), optional`
   頂点の3D法線データ。指定された場合、法線の数は頂点の数と等しくなければなりません。モデルコンパイラは法線を自動的に正規化します。

.. _asset-mesh-texcoord:

:at:`texcoord`: :at-val:`real(2*nvert), optional`
   頂点の2Dテクスチャ座標。0から1の間の数値です。指定された場合、テクスチャ座標ペアの数は頂点の数と等しくなければなりません。

.. _asset-mesh-face:

:at:`face`: :at-val:`int(3*nface), optional`
   メッシュの面。各面は3つの頂点インデックスのシーケンスであり、反時計回りの順序です。インデックスは0からnvert-1の間の整数でなければなりません。

.. _asset-mesh-refpos:

:at:`refpos`: :at-val:`real(3), "0 0 0"`
   3D頂点座標が定義される基準位置。このベクトルは位置から減算されます。

.. _asset-mesh-refquat:

:at:`refquat`: :at-val:`real(4), "1 0 0 0"`
   3D頂点座標と法線が定義される基準方向。このクォータニオンの共役を使用して位置と法線を回転します。モデルコンパイラはクォータニオンを自動的に正規化します。

.. _asset-mesh-builtin:

:at:`builtin`: :at-val:`string, optional`
   メッシュは、 :ref:`params<asset-mesh-params>` で指定されたパラメータのセットからコンパイラによって生成されます。XMLに保存されると、この方法で生成されたメッシュは明示的な頂点に変換されます。Pythonバインディングには、これらのメッシュを生成するための :ref:`便利なメソッド <PyEditConvenience>` が含まれています。利用可能な組み込みタイプ、そのパラメータとセマンティクスは次のとおりです:

   .. image:: images/XMLreference/s.png
      :width: 23%
      :align: right
      :target: https://github.com/google-deepmind/mujoco/blob/main/test/user/testdata/makemesh.xml

   :at-val:`sphere` (subdivision)
      単位正二十面体の繰り返し細分化（「icosphere」）。 :math:`s` 細分化の場合、このメッシュは :math:`V = 2 + 10 \cdot 4^s` 頂点と :math:`F = 20 \cdot 4^s` 面を持ちます。

      **subdivision**: 整数 [0-4]: 正二十面体の面に適用する細分化の数。

   .. image:: images/XMLreference/h.png
      :width: 23%
      :align: right
      :target: https://github.com/google-deepmind/mujoco/blob/main/test/user/testdata/makemesh.xml

   :at-val:`hemisphere` (resolution)
      四辺形投影半球。解像度 :math:`r` の場合、このメッシュは赤道上に :math:`4r` エッジと頂点を持ち、合計 :math:`V = 2 + 2(r+1)(r+2)` 頂点と :math:`F = 4(r+1)(r+2)` 面を持ちます。

      **resolution**: 整数 [0-10]: 1つの半球象限の赤道離散化。

   .. image:: images/XMLreference/c.png
      :width: 23%
      :align: right
      :target: https://github.com/google-deepmind/mujoco/blob/main/test/user/testdata/makemesh.xml

   :at-val:`cone` (nvert, radius)
      z = -1の正多角形とz = 1の与えられた半径の多角形の凸包。半径が1の場合、メッシュは角柱です。半径が0の場合、(0, 0, 1)に単一の頂点が配置され、メッシュは離散円錐です。半径が正の場合、メッシュは切頂された離散円錐です。

      **nvert**: 整数 >= 3: 多角形の頂点数。
      |br| **radius**: 実数 [0, 1]: 上面の半径。

   .. image:: images/XMLreference/ss.png
      :width: 23%
      :align: right
      :target: https://github.com/google-deepmind/mujoco/blob/main/test/user/testdata/makemesh.xml

   :at-val:`supersphere` (resolution, e, n)
      球の一般化。超楕円体としても知られています（半軸の再スケーリングは :ref:`scale<asset-mesh-scale>` 属性によって実行されるため、「supersphere」を使用します）。**n** と **e** パラメータが両方とも1の場合、形状は球です。詳細については `こちら <https://en.wikipedia.org/wiki/Superellipsoid>`__ を参照してください。

      **resolution** 整数 >= 3: 経度と緯度の離散化。
      |br| **e**: 実数 >= 0: 「東西」指数。
      |br| **n**: 実数 >= 0: 「南北」指数。

   .. image:: images/XMLreference/st.png
      :width: 23%
      :align: right
      :target: https://github.com/google-deepmind/mujoco/blob/main/test/user/testdata/makemesh.xml

   :at-val:`supertorus` (resolution, radius, s, t)
      主半径が1で、与えられた副半径を持つトーラスの一般化。**s** と **t** パラメータが両方とも1の場合、形状はトーラスです。詳細については `こちら <https://en.wikipedia.org/wiki/Supertoroid>`__ を参照してください。この形状は本質的に非凸であり、メッシュ衝突に関する :ref:`標準的な注意事項<coDecomposition>` が適用されることに注意してください。

      **resolution** 整数 >= 3: 両方の円周の離散化。
      |br| **radius**: 実数 (0, 1]: トーラスの副半径。
      |br| **s**: 実数 > 0: 副断面の「角張り具合」。
      |br| **t**: 実数 > 0: 主断面の「角張り具合」。

   .. image:: images/XMLreference/w.png
      :width: 23%
      :align: right
      :target: https://github.com/google-deepmind/mujoco/blob/main/test/user/testdata/makemesh.xml

   :at-val:`wedge` (res_phi, res_theta, fov_phi, fov_theta, gamma)
      球座標における単位球殻のスライス。このメッシュは :ref:`触覚センサー<sensor-tactile>` で使用するように設計されており、頂点でデータを報告します。

      **res_phi**: 整数 >= 0: スライスの垂直解像度。
      |br| **res_theta**: 整数 >= 0: スライスの水平解像度。
      |br| **fov_phi**: 実数 (0, 180]: 水平視野角（度）。
      |br| **fov_phi**: 実数 (0, 90): 垂直視野角（度）。
      |br| **gamma**: 実数 [0, 1]: 離散化の中心窩変形。

   :at-val:`plate` (res_x, res_y)
      各次元で与えられた解像度を持つ長方形プレート。このメッシュは :ref:`触覚センサー<sensor-tactile>` で使用するように設計されており、頂点でデータを報告します。

      **res_x**: 整数 > 0: プレートの水平解像度。
      |br| **res_y**: 整数 > 0: プレートの垂直解像度。

.. _asset-mesh-params:

:at:`params`: :at-val:`real(nparam), optional`
   組み込みメッシュの生成に使用されるパラメータ。パラメータの数と型、およびそれらのセマンティクスはメッシュタイプに依存します。詳細については :ref:`mesh/builtin<asset-mesh-builtin>` を参照してください。

.. _asset-mesh-material:

:at:`material`: :at-val:`string, optional`
   独自のマテリアルを指定しないメッシュジオムのフォールバックマテリアル。

.. _mesh-plugin:

:el-prefix:`mesh/` |-| **plugin** |?|
'''''''''''''''''''''''''''''''''''''

このメッシュを :ref:`エンジンプラグイン<exPlugin>` に関連付けます。 :at:`plugin` または :at:`instance` のいずれかが必要です。

.. _mesh-plugin-plugin:

:at:`plugin`: :at-val:`string, optional`
   プラグイン識別子。暗黙的なプラグインのインスタンス化に使用されます。

.. _mesh-plugin-instance:

:at:`instance`: :at-val:`string, optional`
   インスタンス名。明示的なプラグインのインスタンス化に使用されます。



.. _asset-hfield:

:el-prefix:`asset/` |-| **hfield** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

この要素はハイトフィールドアセットを作成し、その後 type "hfield" を持つジオムから参照できるようになります。ハイトフィールド（別名テレインマップ）は、標高データの2次元行列です。データは次の3つの方法のいずれかで指定できます：

#. 標高データをPNGファイルから読み込めます。画像は内部でグレースケールに変換され、各ピクセルの輝度が標高の定義に使用されます。白が高く、黒が低くなります。

#. 標高データを以下に説明するカスタムフォーマットのバイナリファイルから読み込めます。MuJoCoで使用される他のすべての行列と同様に、データの順序はピクセルのように行優先です。データサイズが nrow 行 ncol 列の場合、ファイルは 4*(2+nrow*ncol) バイトでなければなりません：

   ::

              (int32)   nrow
              (int32)   ncol
              (float32) data[nrow*ncol]


#. 標高データはコンパイル時に未定義のまま残すことができます。これは属性 nrow と ncol を指定することで行います。コンパイラは mjModel 内のハイトフィールドデータのスペースを割り当て、それを0に設定します。その後、ユーザーはプログラムまたはセンサーデータを使用して、実行時にカスタムハイトフィールドを生成できます。

| どの方法を使用して標高データを指定しても、コンパイラは常にそれを範囲 [0 1] に正規化します。ただし、データがコンパイル時に未定義のまま残され、後で実行時に生成される場合は、ユーザーが正規化する責任があります。
| ハイトフィールドの位置と方向は、それを参照するジオムによって決定されます。一方、空間的な広がりはハイトフィールドアセット自体が size 属性を介して指定し、参照するジオムでは変更できません（この場合、ジオムのサイズパラメータは無視されます）。同じアプローチが以下のメッシュにも使用されます：位置指定はジオムが行い、サイズ指定はアセットが行います。これは、ハイトフィールドとメッシュには他のジオムには共通でないサイズ設定操作が含まれるためです。
| 衝突検出では、ハイトフィールドは三角柱の結合として扱われます。ハイトフィールドと他のジオム（平面および他のハイトフィールドはサポートされていません）の間の衝突は、まずバウンディングボックスに基づいてジオムと衝突する可能性のある柱のサブグリッドを選択し、次に一般的な凸コライダーを使用して計算されます。ハイトフィールドとジオムの間の可能な接触の数は50に制限されています（ :ref:`mjMAXCONPAIR <glNumeric>` ）。それを超える接触は破棄されます。破棄された接触による貫通を避けるために、ハイトフィールドの空間的特徴は、それが衝突するジオムに比べて大きくする必要があります。

.. _asset-hfield-name:

:at:`name`: :at-val:`string, optional`
   ハイトフィールドの名前で、参照に使用されます。名前が省略され、ファイル名が指定されている場合、ハイトフィールド名はパスと拡張子を除いたファイル名と等しくなります。

.. _asset-hfield-content_type:

:at:`content_type`: :at-val:`string, optional`
   file 属性が指定されている場合、これは読み込まれるファイルの `Media Type <https://www.iana.org/assignments/media-types/media-types.xhtml>`__ （以前は MIME タイプとして知られていた）を設定します。ファイル名拡張子は上書きされます。現在、 ``image/png`` と ``image/vnd.mujoco.hfield`` がサポートされています。

.. _asset-hfield-file:

:at:`file`: :at-val:`string, optional`
   この属性が指定されている場合、標高データは指定されたファイルから読み込まれます。ファイル拡張子が ".png"（大文字小文字を区別しない）の場合、ファイルはPNGファイルとして扱われます。それ以外の場合は、上記のカスタムフォーマットのバイナリファイルとして扱われます。データの行数と列数はファイルの内容から決定されます。ファイルからデータを読み込み、かつ以下の nrow または ncol をゼロ以外の値に設定すると、これらの設定がファイルの内容と一致している場合でも、コンパイルエラーになります。

.. _asset-hfield-nrow:

:at:`nrow`: :at-val:`int, "0"`
   この属性と次の属性は、mjModel 内のハイトフィールドを割り当てるために使用されます。 :at:`elevation` 属性が設定されていない場合、標高データは0に設定されます。この属性は、標高データ行列の行数を指定します。デフォルト値の0は、データがファイルから読み込まれることを意味し、行列のサイズを推測するために使用されます。

.. _asset-hfield-ncol:

:at:`ncol`: :at-val:`int, "0"`
   この属性は、標高データ行列の列数を指定します。

.. _asset-hfield-elevation:

:at:`elevation`: :at-val:`real(nrow*ncol), optional`
   この属性は標高データ行列を指定します。値は、まず最小値を減算し、次に（最大値-最小値）の差で除算することで、自動的に0から1の間に正規化されます（ゼロでない場合）。指定されていない場合、値は0に設定されます。なお、 :ref:`mjModel` および :ref:`mjsHField` のデータの行順序は、XML の順序と反転しています。つまり、下から上になります。

.. _asset-hfield-size:

:at:`size`: :at-val:`real(4), required`
   .. figure:: images/XMLreference/peaks.png
      :width: 350px
      :align: right

   ここでの4つの数値は (radius_x, radius_y, elevation_z, base_z) です。ハイトフィールドは参照するジオムのローカルフレームの中心に配置されます。標高は +Z 方向です。最初の2つの数値は、ハイトフィールドが定義される矩形の X と Y の範囲（または「半径」）を指定します。これは矩形にとっては不自然に見えるかもしれませんが、球体や他のジオムタイプにとっては自然であり、モデル全体で同じ規則を使用することを好みます。3番目の数値は最大標高です。これは [0-1] に正規化された標高データをスケーリングします。したがって、最小標高点は Z=0 にあり、最大標高点は Z=elevation_z にあります。最後の数値は、ハイトフィールドの「ベース」として機能する -Z 方向のボックスの深さです。この自動生成されるボックスがないと、正規化された標高データがゼロの場所でハイトフィールドの厚さがゼロになります。グローバルな片側制約を課す平面とは異なり、ハイトフィールドは通常のジオムの結合として扱われるため、ハイトフィールドの「下」にいるという概念はありません。代わりに、ジオムはハイトフィールドの内側または外側にあります。そのため、内側の部分はゼロでない厚さを持たなければなりません。右の例は、MATLAB の "peaks" 曲面を当社のカスタムハイトフィールドフォーマットで保存し、size = "1 1 1 0.1" でアセットとして読み込んだものです。ボックスの水平サイズは2、最大標高と最小標高の差は1、最小標高点の下に追加されたベースの深さは0.1です。


.. _asset-skin:

:el-prefix:`asset/` |-| **skin** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. _asset-skin-name:
.. _asset-skin-file:
.. _asset-skin-vertex:
.. _asset-skin-texcoord:
.. _asset-skin-face:
.. _asset-skin-inflate:
.. _asset-skin-material:
.. _asset-skin-rgba:
.. _asset-skin-group:

:ref:`スキン<deformable-skin>` は新しいグループ化要素 :ref:`deformable<deformable>` の下に移動されました。ここで指定することもできますが、この機能は現在非推奨であり、将来削除される予定です。



.. _asset-texture:

:el-prefix:`asset/` |-| **texture** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
この要素はテクスチャアセットを作成し、その後 :ref:`マテリアル <asset-material>` アセットから参照され、最終的にテクスチャを適用する必要があるモデル要素から参照されます。

テクスチャデータは、ファイルから読み込むか、コンパイラによってプロシージャルテクスチャとして生成できます。異なるテクスチャタイプには異なるパラメータが必要なため、任意のテクスチャに対して以下の属性のサブセットのみが使用されます。個々の画像ファイルからキューブテクスチャとスカイボックステクスチャを読み込むための規定が提供されています。

現在、テクスチャの読み込みには3つのファイルフォーマットがサポートされています：PNG、KTX、およびカスタム MuJoCo テクスチャフォーマットです。ローダーは、ファイル名の拡張子を使用してどのフォーマットを使用するかを決定し、拡張子が認識されない場合はカスタムフォーマットをデフォルトとします。または、content_type 属性を使用してフォーマットを明示的に指定できます。 ``image/png`` 、 ``image/ktx`` 、または ``image/vnd.mujoco.texture`` のみがサポートされています。

カスタム MuJoCo フォーマットは、次のデータを含むバイナリファイルであると想定されます：

.. code:: Text

       (int32)   width
       (int32)   height
       (byte)    rgb_data[3*width*height]

.. _asset-texture-name:

:at:`name`: :at-val:`string, optional`
   他のすべてのアセットと同様に、テクスチャは参照されるために名前を持たなければなりません。ただし、テクスチャが file 属性を持つ単一のファイルから読み込まれる場合、明示的な名前を省略でき、ファイル名（パスと拡張子を除く）がテクスチャ名になります。解析後の名前が空で、テクスチャタイプが "skybox" でない場合、コンパイラはエラーを生成します。

.. _asset-texture-type:

:at:`type`: :at-val:`[2d, cube, skybox], "cube"`
   この属性は、テクスチャがどのように表現され、オブジェクトにマッピングされるかを決定します。また、残りの属性のどれが関連するかも決定します。キーワードには次の意味があります：

   **cube** タイプは、テクスチャキューブをオブジェクトの上に縮小ラップする効果があります。 :ref:`マテリアル <asset-material>` の texuniform 属性によって提供される調整を除いて、プロセスは自動です。内部的に、GPU はオブジェクトの中心から各ピクセル（またはフラグメント）への光線を構築し、この光線とキューブ表面の交点を見つけ（キューブとオブジェクトは同じ中心を持ちます）、対応するテクスチャ色を使用します。キューブを定義する6つの正方形の画像は同じでも異なっていてもかまいません。同じ場合は、1つのコピーのみが mjModel に格納されます。テクスチャデータを指定するメカニズムは4つあります：

   #. file 属性で指定された単一のファイル（PNG またはカスタム）で、キューブの各面に繰り返される正方形の画像を含みます。これが最も一般的なアプローチです。たとえば、木材の外観を作成することが目標である場合、すべての面に同じ画像を繰り返すだけで十分です。
   #. 6つの正方形がコンパイラによって抽出される複合画像を含む単一のファイル。複合画像のレイアウトは、gridsize 属性と gridlayout 属性によって決定されます。
   #. fileright、fileleft などの属性で指定された6つの個別のファイルで、それぞれ1つの正方形の画像を含みます。
   #. 内部で生成されたプロシージャルテクスチャ。プロシージャルテクスチャのタイプは、builtin 属性によって決定されます。テクスチャデータは、以下に記載されているいくつかのパラメータにも依存します。

   **skybox** タイプはキューブマッピングに非常に似ており、実際、テクスチャデータは完全に同じ方法で指定されます。唯一の違いは、ビジュアライザーがモデルで定義された最初のそのようなテクスチャを使用してスカイボックスをレンダリングすることです。これは、カメラの中心に配置され、常にカメラと一緒に移動する大きなボックスで、サイズは遠方クリッピング平面から自動的に決定されます。アイデアは、スカイボックス上の画像が静止しているように見え、無限に遠くにあるかのように見えることです。このようなテクスチャが通常のオブジェクトに適用されたマテリアルから参照された場合、効果はキューブマップと同等です。ただし、スカイボックスに適した画像は、オブジェクトのテクスチャリングに適していることはほとんどありません。

   **2d** タイプは、 :ref:`テクスチャ座標<asset-mesh-texcoord>` （別名UV座標）を使用して2D画像を3Dオブジェクトにマッピングします。ただし、UV座標はメッシュでのみ使用できます。プリミティブジオムの場合、テクスチャは、ジオムのローカルXY座標を使用してオブジェクト表面にマッピングされ、実質的にZ軸に沿ってテクスチャを投影します。このタイプのマッピングは、平面とハイトフィールドにのみ適しています。なぜなら、それらの上面は常にZ軸を向いているからです。2dテクスチャは、正方形でなければならないキューブテクスチャの側面とは異なり、矩形にすることができます。スケーリングは :ref:`マテリアル <asset-material>` の texrepeat 属性で制御できます。データは単一のファイルから読み込むか、プロシージャルに作成できます。

.. _asset-texture-colorspace:

:at:`colorspace`: :at-val:`[auto, linear, sRGB], "auto"`
   この属性は、テクスチャの色空間を決定します。デフォルト値 ``auto`` は、色空間が画像ファイル自体から決定されることを意味します。ファイルに色空間が定義されていない場合、 ``linear`` が想定されます。

.. _asset-texture-content_type:

:at:`content_type`: :at-val:`string, optional`
   file 属性が指定されている場合、これは読み込まれるファイルの `Media Type <https://www.iana.org/assignments/media-types/media-types.xhtml>`_ （以前は MIME タイプとして知られていた）を設定します。ファイル名拡張子は無視されます。現在、 ``image/png`` 、 ``image/ktx`` 、および ``image/vnd.mujoco.texture`` がサポートされています。

.. _asset-texture-file:

:at:`file`: :at-val:`string, optional`
   この属性が指定され、かつ以下の builtin 属性が "none" に設定されている場合、テクスチャデータは単一のファイルから読み込まれます。ファイルパスについては、 :ref:`compiler <compiler>` の texturedir 属性を参照してください。

.. _asset-texture-gridsize:

:at:`gridsize`: :at-val:`int(2), "1 1"`
   キューブまたはスカイボックステクスチャが単一のファイルから読み込まれる場合、この属性と次の属性は、テクスチャキューブの6つの正方形の側面が単一の画像からどのように取得されるかを指定します。デフォルト設定 "1 1" は、同じ画像がキューブのすべての面に繰り返されることを意味します。それ以外の場合、画像は6つの側面が抽出されるグリッドとして解釈されます。ここでの2つの整数は、グリッドの行数と列数に対応します。各整数は正でなければならず、2つの積は12を超えることはできません。画像の行数と列数は、グリッドの行数と列数の整数倍でなければならず、これらの2つの倍数は等しくなければなりません。そうすることで、抽出される画像が正方形になります。

.. _asset-texture-gridlayout:

:at:`gridlayout`: :at-val:`string, "............"`
   .. figure:: images/XMLreference/skybox.png
      :width: 250px
      :align: right

   キューブまたはスカイボックステクスチャが単一のファイルから読み込まれ、グリッドサイズが "1 1" と異なる場合、この属性は、どのグリッドセルが使用され、キューブのどの側面に対応するかを指定します。オンラインには複合画像として利用可能な多くのスカイボックステクスチャがありますが、同じ規則を使用していないため、それらをデコードするための柔軟なメカニズムを設計しました。ここで指定される文字列は、セット {'.'、'R'、'L'、'U'、'D'、'F'、'B'} の文字で構成されている必要があります。文字数は2つのグリッドサイズの積と等しくなければなりません。グリッドは行優先順序でスキャンされます。'.' 文字は未使用のセルを示します。他の文字は、Right、Left、Up、Down、Front、Back の頭文字です。座標フレームの説明については以下を参照してください。特定の側面の記号が複数回出現する場合、最後の定義が使用されます。特定の側面が省略された場合、rgb1 属性で指定された色で塗りつぶされます。たとえば、以下の砂漠の風景は、gridsize = "3 4" および gridlayout = ".U..LFRB.D.." を使用してスカイボックスまたはキューブマップとして読み込むことができます。マークなしのフル解像度画像ファイルは `こちら <_static/desert.png>`__ からダウンロードできます。

.. _asset-texture-fileright:

.. _asset-texture-fileleft:

.. _asset-texture-fileup:

.. _asset-texture-filedown:

.. _asset-texture-filefront:

.. _asset-texture-fileback:

:at:`fileright`, :at:`fileleft`, :at:`fileup`, :at:`filedown`, :at:`filefront`, :at:`fileback` : string, optional
   これらの属性は、キューブまたはスカイボックステクスチャの6つの側面を個別のファイルから読み込むために使用されますが、file 属性が省略され、builtin 属性が "none" に設定されている場合のみです。これらの属性のいずれかが省略された場合、対応する側面は rgb1 属性で指定された色で塗りつぶされます。ここでの座標フレームは珍しいです。スカイボックスが初期構成でデフォルトのフリーカメラで表示されると、Right、Left、Up、Down の側面は期待される場所に表示されます。Back 側面は視聴者の前に表示されます。なぜなら、視聴者はボックスの真ん中にいて、その背面を向いているからです。ただし、複雑な点があります。MuJoCo では +Z 軸が上を指しますが、既存のスカイボックステクスチャ（設計が簡単ではない）は +Y 軸が上を指すと想定する傾向があります。座標の変更は、ファイル名を変更するだけでは行えません。代わりに、画像の一部を転置または反転する必要があります。この複雑さを避けるために、当社の規則に違反して、スカイボックスを +X 軸の周りに90度回転してレンダリングします。ただし、通常のオブジェクトに対して同じことはできません。したがって、オブジェクトのローカルフレームで表されるスカイボックスとキューブテクスチャの通常のオブジェクトへのマッピングは次のようになります：Right = +X、Left = -X、Up = +Y、Down = -Y、Front = +Z、Back = -Z。

.. _asset-texture-builtin:

:at:`builtin`: :at-val:`[none, gradient, checker, flat], "none"`
   この属性と残りの属性は、プロシージャルテクスチャの生成を制御します。この属性の値が "none" と異なる場合、テクスチャはプロシージャルとして扱われ、ファイル名は無視されます。キーワードには次の意味があります：

   **gradient**
      rgb1 から rgb2 への色のグラデーションを生成します。色空間での補間は、シグモイド関数を介して行われます。キューブおよびスカイボックステクスチャの場合、グラデーションは +Y 軸に沿っています。つまり、スカイボックスレンダリングでは上から下になります。

   **checker**
      rgb1 と rgb2 で指定された交互の色を持つ 2×2 のチェッカーパターンを生成します。これは、地面平面のレンダリングや、回転対称性を持つオブジェクトのマーキングに適しています。2dテクスチャは、必要な回数だけパターンを繰り返すようにスケーリングできることに注意してください。キューブおよびスカイボックステクスチャの場合、チェッカーパターンはキューブの各面に描かれます。

   **flat**
      テクスチャ全体を rgb1 で塗りつぶしますが、キューブおよびスカイボックステクスチャの底面は rgb2 で塗りつぶされます。

.. _asset-texture-rgb1:

:at:`rgb1`: :at-val:`real(3), "0.8 0.8 0.8"`
   プロシージャルテクスチャ生成に使用される最初の色です。この色は、ファイルから読み込まれたキューブおよびスカイボックステクスチャの欠落している側面を塗りつぶすためにも使用されます。この RGB(A) ベクトルおよび他のすべての RGB(A) ベクトルの成分は、範囲 [0 1] でなければなりません。

.. _asset-texture-rgb2:

:at:`rgb2`: :at-val:`real(3), "0.5 0.5 0.5"`
   プロシージャルテクスチャ生成に使用される2番目の色です。

.. _asset-texture-mark:

:at:`mark`: :at-val:`[none, edge, cross, random], "none"`
   プロシージャルテクスチャは、builtin タイプによって決定される色の上に、markrgb 色でマークできます。"edge" は、すべてのテクスチャ画像のエッジがマークされることを意味します。"cross" は、各画像の真ん中に十字がマークされることを意味します。"random" は、ランダムに選択されたピクセルがマークされることを意味します。すべてのマーキングは1ピクセル幅であるため、小さいテクスチャではマーキングがより大きく、より拡散して見えます。

.. _asset-texture-markrgb:

:at:`markrgb`: :at-val:`real(3), "0 0 0"`
   プロシージャルテクスチャマーキングに使用される色です。

.. _asset-texture-random:

:at:`random`: :at-val:`real, "0.01"`
   mark 属性が "random" に設定されている場合、この属性は各ピクセルをオンにする確率を決定します。大きなテクスチャはより多くのピクセルを持ち、ここでの確率は各ピクセルに独立して適用されることに注意してください。したがって、テクスチャサイズと確率を共同で調整する必要があります。グラデーションスカイボックステクスチャと組み合わせて、星のある夜空の外観を作成できます。乱数生成器は固定シードで初期化されます。

.. _asset-texture-width:

:at:`width`: :at-val:`int, "0"`
   プロシージャルテクスチャの幅、つまり画像の列数です。値が大きいほど、通常は高品質の画像が得られますが、場合によっては（たとえば、チェッカーパターン）小さい値で十分です。ファイルから読み込まれたテクスチャの場合、この属性は無視されます。

.. _asset-texture-height:

:at:`height`: :at-val:`int, "0"`
   プロシージャルテクスチャの高さ、つまり画像の行数です。キューブおよびスカイボックステクスチャの場合、この属性は無視され、高さは幅の6倍に設定されます。ファイルから読み込まれたテクスチャの場合、この属性は無視されます。

.. _asset-texture-hflip:

:at:`hflip`: :at-val:`[false, true], "false"`
   true の場合、ファイルから読み込まれた画像は水平方向に反転されます。プロシージャルテクスチャには影響しません。

.. _asset-texture-vflip:

:at:`vflip`: :at-val:`[false, true], "false"`
   true の場合、ファイルから読み込まれた画像は垂直方向に反転されます。プロシージャルテクスチャには影響しません。

.. _asset-texture-nchannel:

:at:`nchannel`: :at-val:`int, "3"`
   テクスチャ画像ファイルのチャンネル数です。これにより、4チャンネルテクスチャ（RGBA）または単一チャンネルテクスチャ（たとえば、粗さや金属性などの物理ベースレンダリングプロパティ用）を読み込むことができます。


.. _asset-material:

:el-prefix:`asset/` |-| **material** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

この要素はマテリアルアセットを作成します。これは :ref:`スキン <deformable-skin>` 、 :ref:`ジオム <body-geom>` 、 :ref:`サイト <body-site>` 、および :ref:`テンドン <tendon>` から参照して、その外観を設定できます。これらの要素はすべてローカルな rgba 属性も持っており、色のみを調整する必要がある場合はより便利です。なぜなら、マテリアルを作成して参照する必要がないからです。マテリアルは、色以外の外観プロパティを調整するのに役立ちます。ただし、マテリアルが作成されたら、すべての外観プロパティがグループ化されるように、マテリアルを使用して色を指定する方が自然です。

.. _asset-material-name:

:at:`name`: :at-val:`string, required`
   マテリアルの名前で、参照に使用されます。

.. _asset-material-class:

:at:`class`: :at-val:`string, optional`
   未指定の属性を設定するためのデフォルトクラスです。

.. _asset-material-texture:

:at:`texture`: :at-val:`string, optional`
   この属性が指定されている場合、マテリアルにはテクスチャが関連付けられています。モデル要素からマテリアルを参照すると、そのテクスチャがその要素に適用されます。この属性の値は、テクスチャアセットの名前であり、テクスチャファイル名ではないことに注意してください。テクスチャはマテリアル定義で読み込むことはできません。代わりに、 :ref:`texture <asset-texture>` 要素を介して明示的に読み込み、ここで参照する必要があります。ここで参照されるテクスチャは、RGB 値を指定するために使用されます。高度なレンダリング（たとえば、物理ベースレンダリング）の場合、より多くのテクスチャタイプを指定する必要があります（たとえば、粗さ、金属性）。この場合、この texture 属性は省略し、テクスチャタイプは :ref:`layer <material-layer>` 子要素を使用して指定する必要があります。ただし、組み込みレンダラーはPBRプロパティをサポートしていないため、これらの高度なレンダリング機能は外部レンダラーを使用する場合にのみ利用できます。

.. _asset-material-texrepeat:

:at:`texrepeat`: :at-val:`real(2), "1 1"`
   この属性は、タイプ "2d" のテクスチャに適用されます。次の属性によって決定されるように、オブジェクトサイズまたは空間単位に対して、テクスチャ画像が何回繰り返されるかを指定します。

.. _asset-material-texuniform:

:at:`texuniform`: :at-val:`[false, true], "false"`
   キューブテクスチャの場合、この属性はキューブマッピングの適用方法を制御します。デフォルト値 "false" は、オブジェクトの実際のサイズを使用してキューブマッピングを直接適用することを意味します。値 "true" は、実際のサイズにスケーリングする前に、テクスチャをユニットオブジェクトにマッピングします（幾何学的プリミティブはレンダラーによってユニットオブジェクトとして作成され、その後スケーリングされます）。場合によっては、これによりテクスチャの外観がより均一になりますが、一般的に、どの設定がより良い結果を生むかは、テクスチャとオブジェクトによって異なります。2dテクスチャの場合、この属性は上記の texrepeat と相互作用します。texrepeat を N とします。デフォルト値 "false" は、2dテクスチャが（z方向の側面の）オブジェクトに対して N 回繰り返されることを意味します。値 "true" は、2dテクスチャがオブジェクトサイズに関係なく、1つの空間単位に対して N 回繰り返されることを意味します。

.. _asset-material-emission:

:at:`emission`: :at-val:`real, "0"`
   OpenGL のエミッションは RGBA 形式ですが、スカラー設定のみを提供します。OpenGL エミッションベクトルの RGB 成分は、ここで指定された値で乗算されたマテリアル色の RGB 成分です。アルファ成分は1です。

.. _asset-material-specular:

:at:`specular`: :at-val:`real, "0.5"`
   OpenGL のスペキュラリティは RGBA 形式ですが、スカラー設定のみを提供します。OpenGL スペキュラリティベクトルの RGB 成分はすべて、ここで指定された値と等しくなります。アルファ成分は1です。この値は範囲 [0 1] でなければなりません。

.. _asset-material-shininess:

:at:`shininess`: :at-val:`real, "0.5"`
   OpenGL のシャイニネスは0から128の間の数値です。ここで指定された値は、OpenGL に渡される前に128で乗算されるため、範囲 [0 1] でなければなりません。大きい値は、より狭いスペキュラーハイライトに対応します（したがって、ハイライトの全体的な量を減らしますが、視覚的にはより顕著になります）。これは、スペキュラリティ設定と相互作用します。詳細については、OpenGL ドキュメントを参照してください。

.. _asset-material-reflectance:

:at:`reflectance`: :at-val:`real, "0"`
   この属性は範囲 [0 1] でなければなりません。値が0より大きく、マテリアルが平面またはボックスジオムに適用されている場合、レンダラーは反射をシミュレートします。値が大きいほど、反射が強くなります。ボックスの場合、ローカル +Z 軸の方向の面のみが反射します。反射を適切にシミュレートするにはレイトレーシングが必要ですが、これは（まだ）リアルタイムでは実行できません。代わりに、ステンシルバッファと適切な投影を使用しています。モデル内の最初の反射ジオムのみがそのようにレンダリングされます。これにより、すべてのジオムを通る1回の追加レンダリングパスが追加されます。これは、影をキャストする各ライトによって追加される追加のレンダリングパスに加えてです。

.. _asset-material-metallic:

:at:`metallic`: :at-val:`real, "-1"`
   この属性は、マテリアル全体に適用される均一な金属性係数に対応します。この属性は、MuJoCo のネイティブレンダラーでは効果がありませんが、物理ベースレンダラーでシーンをレンダリングする場合に役立ちます。この場合、非負の値が指定されると、この金属性値は、サンプリングされた金属性テクスチャ値で乗算して、マテリアルの最終的な金属性を取得する必要があります。

.. _asset-material-roughness:

:at:`roughness`: :at-val:`real, "-1"`
   この属性は、マテリアル全体に適用される均一な粗さ係数に対応します。この属性は、MuJoCo のネイティブレンダラーでは効果がありませんが、物理ベースレンダラーでシーンをレンダリングする場合に役立ちます。この場合、非負の値が指定されると、この粗さ値は、サンプリングされた粗さテクスチャ値で乗算して、マテリアルの最終的な粗さを取得する必要があります。

.. _asset-material-rgba:

:at:`rgba`: :at-val:`real(4), "1 1 1 1"`
   マテリアルの色と透明度です。すべての成分は範囲 [0 1] でなければなりません。テクスチャ色（割り当てられている場合）とここで指定された色は、成分ごとに乗算されることに注意してください。したがって、デフォルト値 "1 1 1 1" は、テクスチャを変更しないままにする効果があります。独自のローカル rgba 属性を定義するモデル要素にマテリアルが適用されると、ローカル定義が優先されます。この「ローカル」定義は、実際にはデフォルトクラスから取得される可能性があることに注意してください。残りのマテリアルプロパティは常に適用されます。

.. _material-layer:

:el-prefix:`material/` |-| **layer** |m|
''''''''''''''''''''''''''''''''''''''''

マテリアルの外観を指定するために複数のテクスチャが必要な場合、 :ref:`texture <asset-material-texture>` 属性は使用できず、代わりに :el:`layer` 子要素を使用する必要があります。 :at:`texture` 属性と :el:`layer` 子要素の両方を指定するとエラーになります。

.. _material-layer-texture:

:at:`texture`: :at-val:`string, required`
   :ref:`texture <asset-material-texture>` 属性と同様に、テクスチャの名前です。

.. _material-layer-role:

:at:`role`: :at-val:`string, required`
   テクスチャの役割です。有効な値、期待されるチャンネル数、および役割のセマンティクスは次のとおりです：

   .. list-table::
      :widths: 1 1 8
      :header-rows: 1

      * - 値
        - チャンネル
        - 説明
      * - :at:`rgb`
        - 3
        - ベースカラー / アルベド [red, green, blue]
      * - :at:`normal`
        - 3
        - バンプマップ（表面法線）
      * - :at:`occlusion`
        - 1
        - アンビエントオクルージョン
      * - :at:`roughness`
        - 1
        - 粗さ
      * - :at:`metallic`
        - 1
        - 金属性
      * - :at:`opacity`
        - 1
        - 不透明度（アルファチャンネル）
      * - :at:`emissive`
        - 4
        - RGB 光放射強度、4番目のチャンネルに露出ウェイト
      * - :at:`orm`
        - 3
        - パック済み3チャンネル [occlusion, roughness, metallic]
      * - :at:`rgba`
        - 4
        - パック済み4チャンネル [red, green, blue, alpha]

.. _asset-model:

:el-prefix:`asset/` |-| **model** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
この要素は、現在のモデルで :ref:`アタッチメント<body-attach>` に使用できる他の MJCF モデルを指定します。

.. _asset-model-name:

:at:`name`: :at-val:`string, optional`
   :ref:`attach<body-attach>` で参照するために使用されるサブモデルの名前です。指定されていない場合、 :ref:`モデル名<mujoco-model>` が使用されます。

.. _asset-model-file:

:at:`file`: :at-val:`string, required`
   サブモデルが読み込まれるファイルです。サブモデルは有効な MJCF モデルでなければなりません。

.. _asset-model-content_type:

:at:`content_type` :at-val:`string, optional`
   モデルに読み込まれるファイルタイプです。現在、text/xml のみがサポートされています。


.. _body:

**(world)body** |R|
~~~~~~~~~~~~~~~~~~~

この要素は、ネストを介して :ref:`キネマティックツリー <CTree>` を構築するために使用されます。要素 :el:`worldbody` はトップレベルのボディに使用され、要素 :el:`body` は他のすべてのボディに使用されます。トップレベルのボディは制限されたタイプのボディです：子要素 :ref:`inertial <body-inertial>` および :ref:`joint <body-joint>` を持つことができず、また属性も持つことができません。これは、残りのキネマティックツリーが定義される世界フレームの原点に対応します。そのボディ名は自動的に "world" として定義されます。

.. _body-name:

:at:`name`: :at-val:`string, optional`
   ボディの名前です。

.. _body-childclass:

:at:`childclass`: :at-val:`string, optional`
   この属性が存在する場合、デフォルトクラスを許可するすべての子孫要素は、ここで指定されたクラスを使用します。ただし、子孫要素自身がクラスを指定するか、ネストされたボディとフレームのチェーンに沿って childclass 属性を持つ別のボディまたはフレームが出現する場合を除きます。 :ref:`CDefault` を参照してください。

.. _body-mocap:

:at:`mocap`: :at-val:`[false, true], "false"`
   この属性が "true" の場合、ボディは mocap ボディとしてラベル付けされます。これは、world ボディの子であり、ジョイントを持たないボディに対してのみ許可されます。このようなボディは動力学の観点からは固定されていますが、それにもかかわらず、順運動学は各タイムステップでフィールド ``mjData.mocap_{pos,quat}`` からそれらの位置と方向を設定します。これらの配列のサイズは、モデル内の mocap ボディの数に一致するようにコンパイラによって調整されます。このメカニズムは、モーションキャプチャデータをシミュレーションにストリーミングするために使用できます。Mocap ボディは、動的シミュレーションモードでも、インタラクティブビジュアライザーでのマウス摂動を介して移動できます。これは、位置と方向を調整可能な小道具を作成するのに役立ちます。

.. _body-pos:

:at:`pos`: :at-val:`real(3), optional`
   親座標フレーム内のボディフレームの3D位置です。未定義の場合、デフォルトは (0,0,0) です。

.. _body-quat:

.. _body-axisangle:

.. _body-xyaxes:

.. _body-zaxis:

.. _body-euler:

:at:`quat`, :at:`axisangle`, :at:`xyaxes`, :at:`zaxis`, :at:`euler`
   :ref:`COrientation` を参照してください。

.. _body-gravcomp:

:at:`gravcomp`: :at-val:`real, "0"`
  ボディの重量の割合として指定される、重力補償力です。この属性は、ボディの重心に適用される上向きの力を作成し、重力の力に対抗します。例として、値 ``1`` はボディの重量と等しい上向きの力を作成し、重力を正確に補償します。 ``1`` より大きい値は、正味の上向きの力または浮力効果を生み出します。

.. _body-sleep:

:at:`sleep`: :at-val:`[auto, never, allowed, init], "auto"`
   このボディの下のツリーの :ref:`スリープ<Sleeping>` ポリシーです。この属性は、キネマティック :ref:`ツリー<ElemTree>` のルートである移動ボディに対してのみサポートされています。デフォルトの :at-val:`auto` の場合、コンパイラは次のようにスリープポリシーを設定します：

   - アクチュエータの影響を受けるツリーはスリープが許可されません（上書き可能）。
   - ゼロでない剛性と減衰を持つテンドンで接続されているツリーはスリープが許可されません（上書き可能）。
   - 3つ以上のツリーを接続するテンドンで接続されているツリーはスリープが許可されません（上書き不可）。
   - :ref:`フレックス<ElemFlex>` はスリープが許可されません（上書き不可）。
   - 他のすべてのツリーはスリープが許可されます（上書き可能）。

   ポリシー :at-val:`never` および :at-val:`allowed` は、自動コンパイラポリシーのユーザー上書きを構成します。

   :at-val:`init` スリープポリシーはユーザーによってのみ指定でき、「このツリーをスリープ状態として初期化する」ことを意味します。このポリシーは :ref:`mj_resetData` および :ref:`mj_makeData` で実装され、デフォルト構成にのみ適用されます。 :ref:`キーフレーム<keyframe>` がスリープ中のツリーの構成を変更する（またはゼロでない速度を割り当てる）場合、それはウェイクアップされます。このポリシーは、自動スリープメカニズムが起動するのを待つことが高コストになる非常に大きなモデルに役立ちます。スリープとして初期化されたツリーは、深い貫通や空中などの不安定な構成に配置できますが、ウェイクアップされたときにのみ移動します。また、このポリシーは失敗する可能性があることに注意してください。たとえば、sleep="init" とマークされたツリーがそのようにマークされていないツリーと接触している場合（つまり、同じ :ref:`アイランド<soIsland>` にある場合）、ツリーをスリープ状態にすることは不可能です。そのような `モデル <https://github.com/google-deepmind/mujoco/blob/main/test/engine/testdata/sleep/init_island_fail.xml>`__ はコンパイルエラーを引き起こします。

   詳細については、 :ref:`実装ノート<siSleep>` を参照してください。

.. _body-user:

:at:`user`: :at-val:`real(nbody_user), "0 0 ..."`
   :ref:`CUser` を参照してください。


.. _body-inertial:

:el-prefix:`body/` |-| **inertial** |?|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

この要素は、ボディの質量と慣性プロパティを指定します。この要素が特定のボディに含まれていない場合、慣性プロパティはボディに付属するジオムから推測されます。コンパイルされた MJCF モデルが保存されると、XML ライターは、ジオムから推測された場合でも、この要素を使用して慣性プロパティを明示的に保存します。慣性フレームは、その中心がボディの重心と一致し、その軸がボディの主慣性軸と一致するようになっています。したがって、慣性行列はこのフレームで対角化されています。


.. _body-inertial-pos:

:at:`pos`: :at-val:`real(3), required`
   慣性フレームの位置です。慣性プロパティがジオムから推測できる場合でも、この属性は必須です。これは、 :el:`inertial` 要素自体の存在が自動推測メカニズムを無効にするためです。

.. _body-inertial-quat:

.. _body-inertial-axisangle:

.. _body-inertial-xyaxes:

.. _body-inertial-zaxis:

.. _body-inertial-euler:

:at:`quat`, :at:`axisangle`, :at:`xyaxes`, :at:`zaxis`, :at:`euler`
   慣性フレームの方向です。 :ref:`COrientation` を参照してください。

.. _body-inertial-mass:

:at:`mass`: :at-val:`real, required`
   ボディの質量です。負の値は許可されていません。MuJoCo は、一般化座標での慣性行列が正定値であることを要求します。これは、一部のボディが質量ゼロである場合でも達成できることがあります。しかし、一般的に無質量ボディを使用する理由はありません。このようなボディは、ジョイントを組み合わせることができないという制限を回避するため、またはセンサーやカメラを取り付けるために、他のエンジンでよく使用されます。MuJoCo では、プリミティブジョイントタイプを組み合わせることができ、より効率的な取り付けメカニズムであるサイトがあります。

.. _body-inertial-diaginertia:

:at:`diaginertia`: :at-val:`real(3), optional`
   慣性フレームに対する相対的なボディ慣性を表す、対角慣性行列です。この属性が省略された場合、次の属性が必須になります。

.. _body-inertial-fullinertia:

:at:`fullinertia`: :at-val:`real(6), optional`
   完全な慣性行列 M です。M は 3×3 で対称であるため、次の順序で6つの数値のみを使用して指定されます：M(1,1)、M(2,2)、M(3,3)、M(1,2)、M(1,3)、M(2,3)。コンパイラは M の固有値分解を計算し、フレームの方向と対角慣性をそれに応じて設定します。非正の固有値が検出された場合（つまり、M が正定値でない場合）、コンパイルエラーが生成されます。


.. _body-joint:

:el-prefix:`body/` |-| **joint** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

この要素はジョイントを作成します。 :ref:`キネマティックツリー <CTree>` で説明されているように、ジョイントは、それが定義されているボディとボディの親の間に運動自由度を作成します。同じボディに複数のジョイントが定義されている場合、対応する空間変換（親フレームに対するボディフレームの）が順番に適用されます。ジョイントが定義されていない場合、ボディは親に溶接されます。ジョイントは world ボディでは定義できません。実行時に、モデルで定義されたすべてのジョイントの位置と方向は、キネマティックツリーに出現する順序で、ベクトル ``mjData.qpos`` に格納されます。線速度と角速度は、ベクトル ``mjData.qvel`` に格納されます。これら2つのベクトルは、フリージョイントまたはボールジョイントが使用される場合、異なる次元を持ちます。なぜなら、そのようなジョイントは回転を単位クォータニオンとして表すためです。

.. _body-joint-name:

:at:`name`: :at-val:`string, optional`
   ジョイントの名前です。

.. _body-joint-class:

:at:`class`: :at-val:`string, optional`
   未指定の属性を設定するためのデフォルトクラスです。

.. _body-joint-type:

:at:`type`: :at-val:`[free, ball, slide, hinge], "hinge"`
   ジョイントのタイプです。キーワードには次の意味があります：
   **free** タイプは、3つの並進自由度とそれに続く3つの回転自由度を持つフリー「ジョイント」を作成します。言い換えれば、ボディを浮遊させます。回転は単位クォータニオンとして表されます。このジョイントタイプは、world ボディの子であるボディでのみ許可されます。フリージョイントが定義されている場合、ボディで他のジョイントを定義することはできません。残りのジョイントタイプとは異なり、フリージョイントはボディフレーム内に位置を持ちません。代わりに、ジョイントの位置はボディフレームの中心と一致すると想定されます。したがって、実行時には、フリージョイントの位置と方向データは、ボディフレームのグローバル位置と方向に対応します。フリージョイントには制限を設定できません。

   **ball** タイプは、3つの回転自由度を持つボールジョイントを作成します。回転は単位クォータニオンとして表されます。クォータニオン (1,0,0,0) は、モデルが定義される初期構成に対応します。他のクォータニオンは、この初期構成に対する3D回転として解釈されます。回転は、 :ref:`pos<body-joint-pos>` 属性によって定義される点の周りに行われます。ボディにボールジョイントがある場合、他の回転ジョイント（ボールまたはヒンジ）を持つことはできません。ボールジョイントとスライドジョイントを同じボディで組み合わせることは許可されています。

   **slide** タイプは、1つの並進自由度を持つスライドまたはプリズマティックジョイントを作成します。このようなジョイントは、位置とスライド方向によって定義されます。シミュレーション目的では方向のみが必要です。ジョイントの位置はレンダリング目的で使用されます。

   **hinge** タイプは、1つの回転自由度を持つヒンジジョイントを作成します。回転は、指定された位置を通る指定された軸の周りで行われます。これは最も一般的なタイプのジョイントであり、したがってデフォルトです。ほとんどのモデルは、ヒンジジョイントとフリージョイントのみを含みます。

.. _body-joint-group:

:at:`group`: :at-val:`int, "0"`
   ジョイントが属する整数グループです。この属性は、カスタムタグに使用できます。また、ジョイントのグループ全体のレンダリングを有効または無効にするために、ビジュアライザーによって使用されます。

.. _body-joint-pos:

:at:`pos`: :at-val:`real(3), "0 0 0"`
   ジョイントが定義されているボディのフレーム内で指定される、ジョイントの位置です。フリージョイントの場合、この属性は無視されます。

.. _body-joint-axis:

:at:`axis`: :at-val:`real(3), "0 0 1"`
   この属性は、ヒンジジョイントの回転軸とスライドジョイントの並進方向を指定します。フリージョイントとボールジョイントでは無視されます。ここで指定されたベクトルは、その長さが 10E-14 より大きい限り、自動的に単位長に正規化されます。それ以外の場合、コンパイルエラーが生成されます。

.. _body-joint-springdamper:

:at:`springdamper`: :at-val:`real(2), "0 0"`
   両方の数値が正の場合、コンパイラは以下の属性で指定された剛性と減衰の値を上書きし、代わりにこのジョイントの結果として得られる質量・バネ・ダンパーが望ましい時定数（最初の値）と減衰比（2番目の値）を持つように自動的に設定します。これは、モデル参照構成でのジョイント慣性を考慮して行われます。フォーマットは、制約ソルバーの solref パラメータと同じであることに注意してください。

.. _body-joint-solreflimit:

.. _body-joint-solimplimit:

:at:`solreflimit`, :at:`solimplimit`
   ジョイント制限をシミュレートするための制約ソルバーパラメータです。 :ref:`CSolver` を参照してください。

.. _body-joint-solreffriction:

.. _body-joint-solimpfriction:

:at:`solreffriction`, :at:`solimpfriction`
   乾燥摩擦をシミュレートするための制約ソルバーパラメータです。 :ref:`CSolver` を参照してください。

.. _body-joint-stiffness:

:at:`stiffness`: :at-val:`real, "0"`
   ジョイントの剛性です。この値が正の場合、以下の springref で指定される平衡位置を持つバネが作成されます。バネの力は他の受動力と一緒に計算されます。

.. _body-joint-range:

:at:`range`: :at-val:`real(2), "0 0"`
   ジョイントの制限です。制限は、フリージョイントを除くすべてのジョイントタイプに課すことができます。ヒンジジョイントとボールジョイントの場合、範囲は :ref:`compiler <compiler>` の angle 属性に応じて度またはラジアンで指定されます。ボールジョイントの場合、制限は回転軸に関係なく、（参照構成に対する）回転角度に課されます。ボールジョイントでは2番目の範囲パラメータのみが使用されます。最初の範囲パラメータは0に設定する必要があります。詳細については、計算の章の :ref:`制限 <coLimit>` セクションを参照してください。
   |br| :ref:`compiler <compiler>` で :at:`autolimits` が "false" の場合、 :at:`limited` を指定せずにこの属性を設定するとエラーになります。

.. _body-joint-limited:

:at:`limited`: :at-val:`[false, true, auto], "auto"`
   この属性は、ジョイントが制限を持つかどうかを指定します。これは :ref:`range<body-joint-range>` 属性と相互作用します。この属性が "false" の場合、ジョイント制限は無効になります。この属性が "true" の場合、ジョイント制限は有効になります。この属性が "auto" で、 :ref:`compiler <compiler>` で :at:`autolimits` が設定されている場合、range が定義されていればジョイント制限が有効になります。

.. _body-joint-actuatorfrcrange:

:at:`actuatorfrcrange`: :at-val:`real(2), "0 0"`
   このジョイントに作用する総アクチュエータ力をクランプする範囲です。詳細については、 :ref:`CForceRange` を参照してください。スカラージョイント（ヒンジとスライダー）でのみ使用可能で、ボールジョイントとフリージョイントでは無視されます。 |br| コンパイラは、最初の値が2番目の値より小さいことを期待します。 |br| :at:`compiler-autolimits` が "false" の場合、 :at:`actuatorfrclimited` を指定せずにこの属性を設定するとエラーになります。

.. _body-joint-actuatorfrclimited:

:at:`actuatorfrclimited`: :at-val:`[false, true, auto], "auto"`
   この属性は、ジョイントに作用するアクチュエータ力をクランプする必要があるかどうかを指定します。詳細については、 :ref:`CForceRange` を参照してください。スカラージョイント（ヒンジとスライダー）でのみ使用可能で、ボールジョイントとフリージョイントでは無視されます。 |br| この属性は :ref:`actuatorfrcrange<body-joint-actuatorfrcrange>` 属性と相互作用します。この属性が "false" の場合、アクチュエータ力のクランプは無効になります。 "true" の場合、アクチュエータ力のクランプは有効になります。この属性が "auto" で、 :ref:`compiler <compiler>` で :at:`autolimits` が設定されている場合、 :at:`actuatorfrcrange` が定義されていればアクチュエータ力のクランプが有効になります。

.. _body-joint-actuatorgravcomp:

:at:`actuatorgravcomp`: :at-val:`[false, true], "false"`
   このフラグが有効になっている場合、このジョイントに適用される重力補償は、受動力（ ``mjData.qfrc_passive`` ）ではなく、アクチュエータ力（ ``mjData.qfrc_actuator`` ）に追加されます。概念的には、これは重力補償が自然な浮力ではなく制御システムの結果であることを意味します。実際には、このフラグを有効にすることは、ジョイントレベルのアクチュエータ力のクランプが使用される場合に役立ちます。この場合、重力補償を含む、ジョイントに適用される総アクチュエーション力が、指定された制限を超えないことが保証されます。このタイプの力制限の詳細については、 :ref:`CForceRange` および :ref:`actuatorfrcrange<body-joint-actuatorfrcrange>` を参照してください。

.. _body-joint-margin:

:at:`margin`: :at-val:`real, "0"`
   制限がアクティブになる距離閾値です。 :ref:`制約ソルバー <Solver>` は通常、margin パラメータがそれを距離でそうさせる場合でも、制約がアクティブになるとすぐに力を生成することを思い出してください。この属性は、solreflimit および solimplimit と共に使用して、柔らかいジョイント制限をモデル化できます。

.. _body-joint-ref:

:at:`ref`: :at-val:`real, "0"`
   ジョイントの参照位置または角度です。この属性は、スライドジョイントとヒンジジョイントにのみ使用されます。これは、初期モデル構成に対応するジョイント値を定義します。初期構成自体は変更されず、この構成でのジョイントの値のみが変更されることに注意してください。実行時にジョイントが適用する空間変換の量は、 ``mjData.qpos`` に格納されている現在のジョイント値から、 ``mjModel.qpos0`` に格納されているこの参照値を引いたものに等しくなります。これらのベクトルの意味は、概要の章の :ref:`キネマティックツリー <Kinematic>` セクションで説明されています。

.. _body-joint-springref:

:at:`springref`: :at-val:`real, "0"`
   ジョイントのバネ（存在する場合）が平衡を達成するジョイント位置または角度です。上記の ref 属性で指定されたすべてのジョイント参照値を格納するベクトル mjModel.qpos0 と同様に、この属性で指定されたすべてのバネ参照値は、ベクトル mjModel.qpos_spring に格納されます。mjModel.qpos_spring に対応するモデル構成は、mjModel.tendon_lengthspring に格納されているすべてのテンドンのバネ参照長を計算するためにも使用されます。これは、 :ref:`テンドン <tendon>` もバネを持つことができるためです。

.. image:: images/XMLreference/armature.gif
   :width: 40%
   :align: right
   :class: only-light
   :target: https://github.com/google-deepmind/mujoco/blob/main/test/engine/testdata/armature_equivalence.xml
.. image:: images/XMLreference/armature_dark.gif
   :width: 40%
   :align: right
   :class: only-dark
   :target: https://github.com/google-deepmind/mujoco/blob/main/test/engine/testdata/armature_equivalence.xml

.. _body-joint-armature:

:at:`armature`: :at-val:`real, "0"`
   ボディの質量によるものではない、ジョイントの動きに関連する追加の慣性です。この追加された慣性は通常、ギア付き伝達によりジョイント自体よりも速く回転する回転子（別名 `アーマチュア <https://en.wikipedia.org/wiki/Armature_(electrical)>`__ ）によるものです。図では、（*左*）アーマチュアボディ（紫色のボックス）を持つ2自由度システムを、 :ref:`ジョイント等式制約<equality-joint>` を使用してギア比 :math:`3` で振り子に結合したものと、（*右*）等価な :at:`armature` を持つシンプルな1自由度振り子を比較しています。ギア比は力と長さの両方を乗算するため2回現れるため、効果は「反映された慣性」として知られ、等価な値は回転ボディの慣性に *ギア比の2乗* を乗じたものです。この場合は :math:`9=3^2` です。値は、このジョイントによって作成されるすべての自由度に適用されます。

   ギア付き伝達を持つジョイントのリアリズムを向上させることに加えて、正の :at:`armature` は、小さな値であってもシミュレーションの安定性を大幅に向上させます。安定性の問題に遭遇した場合の推奨される可能な修正です。

.. _body-joint-damping:

:at:`damping`: :at-val:`real, "0"`
   このジョイントによって作成されるすべての自由度に適用される減衰です。制約ソルバーによって計算される摩擦損失とは異なり、減衰は単に速度に対して線形な力です。これは受動力に含まれます。このシンプルさにもかかわらず、大きな減衰値は数値積分器を不安定にする可能性があるため、当社のオイラー積分器は減衰を陰的に処理します。計算の章の :ref:`積分 <geIntegration>` を参照してください。

.. _body-joint-frictionloss:

:at:`frictionloss`: :at-val:`real, "0"`
   乾燥摩擦による摩擦損失です。この値は、このジョイントによって作成されるすべての自由度で同じです。意味的には、摩擦損失はフリージョイントには意味がありませんが、コンパイラはそれを許可します。摩擦損失を有効にするには、この属性を正の値に設定します。

.. _body-joint-user:

:at:`user`: :at-val:`real(njnt_user), "0 0 ..."`
   :ref:`CUser` を参照してください。


.. _body-freejoint:

:el-prefix:`body/` |-| **freejoint** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

この要素は、唯一の属性が :at:`name` と :at:`group` であるフリージョイントを作成します。 :el:`freejoint` 要素は、以下のXMLの短縮表記です。

.. code-block:: xml

   <joint type="free" stiffness="0" damping="0" frictionloss="0" armature="0"/>

このジョイントは明らかに :ref:`joint <body-joint>` 要素でも作成できますが、デフォルトのジョイント設定がこれに影響を与える可能性があります。これは通常望ましくありません。物理的なフリーボディは、ゼロでない剛性、減衰、摩擦、またはアーマチュアを持たないからです。この問題を回避するために :el:`freejoint` 要素が導入されました。これにより、ジョイントのデフォルトが *継承されない* ことが保証されます。XMLモデルが保存されると、これは :at:`free` タイプの通常のジョイントとして現れます。


.. _body-freejoint-name:

:at:`name`: :at-val:`string, optional`
   ジョイントの名前です。

.. _body-freejoint-group:

:at:`group`: :at-val:`int, "0"`
   ジョイントが属する整数グループです。この属性はカスタムタグに使用できます。また、ビジュアライザーがジョイントグループ全体のレンダリングを有効化・無効化するためにも使用されます。

.. _body-freejoint-align:

:at:`align`: :at-val:`[false, true, auto], "auto"`
   :at-val:`true` に設定されている場合、ボディフレームとフリージョイントは自動的に慣性フレームと整列されます。 :at-val:`false` に設定されている場合、整列は行われません。 :at-val:`auto` に設定されている場合、コンパイラの :ref:`alignfree<compiler-alignfree>` グローバル属性が尊重されます。

   慣性フレーム整列は最適化であり、フリージョイントと子ボディを持たないボディ（「単純なフリーボディ」）にのみ適用されます。この整列により、6x6慣性行列が対角化され、バイアス力が最小化され、より高速で安定したシミュレーションが実現されます。この挙動は厳密な改善ですが、フリージョイントのセマンティクスを変更するため、古いバージョン（例えば :ref:`キーフレーム<keyframe>` など）で保存された ``qpos`` と ``qvel`` の値が無効になります。

   :at:`align` 属性はXMLに保存されないことに注意してください。代わりに、単純なフリーボディとその子の姿勢が変更され、ボディフレームと慣性フレームが整列されるようになります。

.. _body-geom:

:el-prefix:`body/` |-| **geom** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

この要素はジオムを作成し、そのジオムが定義されているボディに剛的に取り付けます。複数のジオムを同じボディに取り付けることができます。実行時には、それらはボディの外観と衝突特性を決定します。コンパイル時には、 :ref:`inertial <body-inertial>` 要素の存在と :ref:`compiler <compiler>` の inertiafromgeom 属性の設定に応じて、ボディの慣性特性も決定する場合があります。これは、ボディに取り付けられたすべてのジオムの質量と慣性を合計することによって行われます。ただし、ジオムグループが :ref:`compiler <compiler>` の inertiagrouprange 属性で指定された範囲内にある場合に限ります。ジオムの質量と慣性は、ジオム形状、指定された密度またはジオム質量（これは密度を意味します）、および一様密度の仮定を使用して計算されます。

ジオムは物理シミュレーションに厳密には必要ありません。ボディとジョイントのみを持つモデルを作成してシミュレートすることができます。このようなモデルでも、等価慣性ボックスを使用してボディを表現することで可視化できます。ただし、接触力がこのようなシミュレーションから欠落します。このようなモデルの使用は推奨されませんが、これが可能であることを知ることは、MuJoCoにおけるボディとジオムの役割を明確にするのに役立ちます。

.. _body-geom-name:

:at:`name`: :at-val:`string, optional`
   ジオムの名前です。

.. _body-geom-class:

:at:`class`: :at-val:`string, optional`
   指定されていない属性を設定するためのデフォルトクラスです。

.. _body-geom-type:

:at:`type`: :at-val:`[plane, hfield, sphere, capsule, ellipsoid, cylinder, box, mesh, sdf], "sphere"`
   幾何学形状のタイプです。キーワードの意味は以下の通りです：**plane** タイプは、衝突検出の目的で無限平面を定義します。これは、ワールドボディまたはワールドの静的な子にのみ取り付けることができます。平面は、pos 属性で指定された点を通過します。これはジオムのローカルフレームのZ軸に対して垂直です。+Z方向は空間に対応します。したがって、デフォルトの位置と方向 (0,0,0) と (1,0,0,0) は、Z=0の高度にある地面平面を作成し、+Zが世界の垂直方向になります（これがMuJoCoの慣例です）。平面は無限であるため、平面内の他のどの点を使用しても定義できたはずです。しかし、指定された位置はレンダリングに関して追加の意味を持ちます。最初の2つのサイズパラメータのいずれかが正の場合、平面は有限サイズの長方形としてレンダリングされます（正の次元で）。この長方形は指定された位置を中心にしています。3つのサイズパラメータが必要です。最初の2つは、XおよびY軸に沿った長方形のハーフサイズを指定します。3番目のサイズパラメータは異常です：レンダリング目的の平面のグリッド分割の間隔を指定します。この分割はワイヤーフレームレンダリングモードで表示されますが、一般的には地面平面上にグリッドを描画するために使用すべきではありません（その目的にはテクスチャを使用すべきです）。代わりに、それらの役割は、ボックスのレンダリングに使用される分割と同様に、照明と影を改善することです。平面を背面から見ると、自動的に半透明になります。平面とボックスの+Z面のみが反射を表示できます。ただし、ジオムに適用される :ref:`マテリアル <asset-material>` が正の反射を持つ場合に限ります。無限平面をレンダリングするには、最初の2つのサイズパラメータをゼロに設定します。

   **hfield** タイプはハイトフィールドジオムを定義します。ジオムは、以下の hfield 属性で目的のハイトフィールドアセットを参照する必要があります。ジオムの位置と方向は、ハイトフィールドの位置と方向を設定します。ジオムのサイズは無視され、代わりにハイトフィールドアセットのサイズパラメータが使用されます。 :ref:`hfield <asset-hfield>` 要素の説明を参照してください。平面と同様に、ハイトフィールドジオムはワールドボディまたはワールドの静的な子にのみ取り付けることができます。

   **sphere** タイプは球を定義します。これと次の4つのタイプは、組み込みの幾何学プリミティブに対応します。これらのプリミティブは、衝突検出の目的で解析的表面として扱われ、多くの場合、カスタムのペアワイズ衝突ルーチンに依存しています。平面、球、カプセル、およびボックスのみを含むモデルは、衝突検出の観点から最も効率的です。他のジオムタイプは汎用凸コライダーを呼び出します。球はジオムの位置を中心にしています。サイズパラメータは1つだけ使用され、球の半径を指定します。幾何学プリミティブのレンダリングは、 :ref:`quality <visual-quality>` で密度を調整できる自動生成メッシュで行われます。球メッシュは緯度と経度の線に沿って三角形分割され、Z軸が北極と南極を通過します。これは、フレームの方向を可視化するためにワイヤーフレームモードで役立ちます。

   **capsule** タイプはカプセルを定義します。これは2つの半球でキャップされた円筒です。これはジオムのフレームのZ軸に沿って方向付けられています。ジオムフレームが通常の方法で指定されている場合、2つのサイズパラメータが必要です：カプセルの半径の後に円筒部分のハーフハイトが続きます。ただし、カプセルと円筒は、コネクタとしても考えることができ、以下の fromto 属性を使用した代替仕様を可能にします。その場合、必要なサイズパラメータは1つだけで、カプセルの半径です。

   **ellipsoid** タイプは楕円体を定義します。これは、ローカルフレームのX、Y、Z軸に沿って個別にスケールされた球です。3つのサイズパラメータが必要で、3つの半径に対応します。楕円体は滑らかですが、その衝突は汎用凸コライダーを介して処理されることに注意してください。唯一の例外は平面-楕円体の衝突で、これは解析的に計算されます。

   **cylinder** タイプは円筒を定義します。2つのサイズパラメータが必要です：円筒の半径とハーフハイトです。円筒はジオムのフレームのZ軸に沿って方向付けられています。代わりに、以下の fromto 属性で指定することもできます。

   **box** タイプはボックスを定義します。3つのサイズパラメータが必要で、ジオムのフレームのX、Y、Z軸に沿ったボックスのハーフサイズに対応します。ボックス-ボックスの衝突は最大8つの接触点を生成できることに注意してください。

   **mesh** タイプはメッシュを定義します。ジオムは、mesh 属性で目的のメッシュアセットを参照する必要があります。メッシュアセットは他のジオムタイプから参照することもでき、その場合プリミティブ形状がフィッティングされます。以下を参照してください。サイズはメッシュアセットによって決定され、ジオムのサイズパラメータは無視されます。他のすべてのジオムとは異なり、コンパイル後のメッシュジオムの位置と方向は、ここで対応する属性の設定と等しくありません。代わりに、メッシュアセットをその独自の座標フレームで中心化および整列させるために必要だった平行移動と回転によってオフセットされます。 :ref:`mesh <asset-mesh>` 要素の中心化と整列に関する議論を思い出してください。

   **sdf** タイプは符号付き距離場（SDF、符号付き距離関数とも呼ばれます）を定義します。SDFを可視化するには、 :ref:`mesh/plugin <mesh-plugin>` 属性を使用してカスタムメッシュを指定する必要があります。SDFジオメトリを含むモデルの例については、 `model/plugin/sdf/ <https://github.com/google-deepmind/mujoco/tree/main/model/plugin/sdf>`__ ディレクトリを参照してください。SDFプラグインの詳細については、 :ref:`拡張の章<exWriting>` を参照してください。

.. _body-geom-contype:

:at:`contype`: :at-val:`int, "1"`
   この属性と次の属性は、動的に生成される接触ペアの接触フィルタリングに使用される32ビット整数ビットマスクを指定します。Computation章の :ref:`Collision` を参照してください。2つのジオムは、一方のジオムのcontypeがもう一方のジオムのconaffinityと互換性がある場合、またはその逆の場合に衝突できます。互換性とは、2つのビットマスクに共通のビットが1に設定されていることを意味します。

.. _body-geom-conaffinity:

:at:`conaffinity`: :at-val:`int, "1"`
   接触フィルタリング用のビットマスクです。上記のcontypeを参照してください。

.. _body-geom-condim:

:at:`condim`: :at-val:`int, "3"`
   動的に生成される接触ペアの接触空間の次元性は、2つの参加ジオムのcondim値の最大値に設定されます。Computation章の :ref:`coContact` を参照してください。許可される値とその意味は次のとおりです：

   +--------+----------------------------------------------------------------------------------------------------------+
   | condim | 説明                                                                                                     |
   +========+==========================================================================================================+
   | 1      | 摩擦のない接触。                                                                                         |
   +--------+----------------------------------------------------------------------------------------------------------+
   | 3      | 通常の摩擦接触、接線平面での滑りに対抗します。                                                           |
   +--------+----------------------------------------------------------------------------------------------------------+
   | 4      | 摩擦接触、接線平面での滑りと接触法線周りの回転に対抗します。これは、柔らかい接触をモデル化するのに     |
   |        | 役立ちます（接触貫入とは独立して）。                                                                     |
   +--------+----------------------------------------------------------------------------------------------------------+
   | 6      | 摩擦接触、接線平面での滑り、接触法線周りの回転、および接線平面の2つの軸周りの回転に対抗します。後者の   |
   |        | 摩擦効果は、物体が無限に転がるのを防ぐのに役立ちます。                                                   |
   +--------+----------------------------------------------------------------------------------------------------------+

.. _body-geom-group:

:at:`group`: :at-val:`int, "0"`
   この属性は、ジオムが属する整数グループを指定します。物理への唯一の影響はコンパイル時にあり、グループに基づいて選択されたジオムからボディの質量と慣性が推測されます。 :ref:`compiler <compiler>` の inertiagrouprange 属性を参照してください。実行時には、この属性はビジュアライザーがジオムグループ全体のレンダリングを有効化・無効化するために使用されます。デフォルトでは、グループ0、1、2が可視で、他のすべてのグループは不可視です。group属性は、カスタム計算のタグとしても使用できます。

.. _body-geom-priority:

:at:`priority`: :at-val:`int, "0"`
   ジオムの優先度は、2つの衝突ジオムの特性が接触の特性を形成するためにどのように組み合わされるかを決定します。これはsolmix属性と相互作用します。 :ref:`CContact` を参照してください。

.. _body-geom-size:

:at:`size`: :at-val:`real(3), "0 0 0"`
   ジオムのサイズパラメータです。必要なパラメータの数とその意味は、type属性で文書化されているように、ジオムタイプに依存します。ここでは要約のみを示します。必要なすべてのサイズパラメータは正である必要があります。内部デフォルトは無効な設定に対応します。非メッシュジオムタイプがメッシュを参照する場合、そのタイプの幾何学プリミティブがメッシュにフィットされることに注意してください。その場合、サイズはメッシュから取得され、ジオムのサイズパラメータは無視されます。したがって、以下の表の必要なサイズパラメータの数と説明は、メッシュを参照しないジオムにのみ適用されます。

   +---------+--------+------------------------------------------------------------------------------------------------+
   | タイプ  | 数     | 説明                                                                                           |
   +=========+========+================================================================================================+
   | plane   | 3      | Xハーフサイズ；Yハーフサイズ；レンダリング用の正方形グリッド線間の間隔。XまたはYハーフサイズの |
   |         |        | いずれかが0の場合、平面はサイズが0の次元で無限としてレンダリングされます。                      |
   +---------+--------+------------------------------------------------------------------------------------------------+
   | hfield  | 0      | ジオムのサイズは無視され、代わりにハイトフィールドのサイズが使用されます。                       |
   +---------+--------+------------------------------------------------------------------------------------------------+
   | sphere  | 1      | 球の半径。                                                                                     |
   +---------+--------+------------------------------------------------------------------------------------------------+
   | capsule | 1 or 2 | カプセルの半径； :at:`fromto` 仕様を使用しない場合の円筒部分のハーフレングス。                 |
   +---------+--------+------------------------------------------------------------------------------------------------+
   |ellipsoid| 3      | X半径；Y半径；Z半径。                                                                          |
   +---------+--------+------------------------------------------------------------------------------------------------+
   |cylinder | 1 or 2 | 円筒の半径； :at:`fromto` 仕様を使用しない場合の円筒のハーフレングス。                         |
   +---------+--------+------------------------------------------------------------------------------------------------+
   | box     | 3      | Xハーフサイズ；Yハーフサイズ；Zハーフサイズ。                                                  |
   +---------+--------+------------------------------------------------------------------------------------------------+
   | mesh    | 0      | ジオムのサイズは無視され、代わりにメッシュのサイズが使用されます。                               |
   +---------+--------+------------------------------------------------------------------------------------------------+

.. _body-geom-material:

:at:`material`: :at-val:`string, optional`
   指定されている場合、この属性はマテリアルをジオムに適用します。指定されておらず、ジオムのタイプが **mesh** の場合、コンパイラは存在する場合、メッシュアセット :ref:`マテリアル <asset-mesh-material>` を適用します。

   マテリアルはジオムの視覚的プロパティを決定します。唯一の例外は色です：以下の rgba 属性が内部デフォルトと異なる場合、それが優先されますが、残りのマテリアルプロパティは引き続き適用されます。同じマテリアルが複数のジオム（およびサイトとテンドン）から参照され、ユーザーが実行時にそのプロパティの一部を変更した場合、これらの変更はマテリアルを参照するすべてのモデル要素に対して即座に有効になることに注意してください。これは、コンパイラがマテリアルとそのプロパティをmjModelの別の要素として保存し、このマテリアルを使用する要素がそれへの参照のみを保持するためです。

.. _body-geom-rgba:

:at:`rgba`: :at-val:`real(4), "0.5 0.5 0.5 1"`
   マテリアルアセットを作成して参照する代わりに、この属性を使用して色と透明度のみを設定できます。これはマテリアルメカニズムほど柔軟ではありませんが、より便利で、多くの場合十分です。この属性の値が内部デフォルトと異なる場合、マテリアルより優先されます。

.. _body-geom-friction:

:at:`friction`: :at-val:`real(3), "1 0.005 0.0001"`
   動的に生成される接触ペアの接触摩擦パラメータです。最初の数値は、接線平面の両軸に沿って作用する滑り摩擦です。2番目の数値は、接触法線周りに作用するねじり摩擦です。3番目の数値は、接線平面の両軸周りに作用する転がり摩擦です。接触ペアの摩擦パラメータは、 :ref:`接触パラメータ <CContact>` で説明されているように、solmix属性とpriority属性に応じて結合されます。この属性のセマンティクスの説明については、一般的な :ref:`接触<coContact>` セクションを参照してください。

.. _body-geom-mass:

:at:`mass`: :at-val:`real, optional`
   この属性が指定されている場合、以下のdensity属性は無視され、ジオムの密度は、ジオム形状と一様密度の仮定を使用して、指定された質量から計算されます。計算された密度は、ジオムの慣性を取得するために使用されます。ジオムの質量と慣性は、コンパイル中にのみ使用され、必要に応じてボディの質量と慣性を推測することを思い出してください。実行時には、ボディの慣性プロパティのみがシミュレーションに影響を与えます。ジオムの質量と慣性はmjModelに保存されません。

.. _body-geom-density:

:at:`density`: :at-val:`real, "1000"`
   ジオムの質量と慣性を計算するために使用される材料密度です。計算は、ジオム形状と一様密度の仮定に基づいています。内部デフォルトの1000は、SI単位での水の密度です。この属性は、上記のmass属性が指定されていない場合にのみ使用されます。 `shellinertia` が "false"（デフォルト）の場合、densityは質量/体積のセマンティクスを持ちます。 "true" の場合、質量/面積のセマンティクスを持ちます。

.. _body-geom-shellinertia:

:at:`shellinertia` :at-val:`[false, true], "false"`
   trueの場合、ジオムの慣性は、すべての質量が表面に集中していると仮定して計算されます。この場合、 :at:`density` は体積密度ではなく表面密度として解釈されます。この属性はプリミティブジオムにのみ適用され、メッシュでは無視されます。メッシュの表面慣性は、 :ref:`asset/mesh/inertia<asset-mesh-inertia>` 属性を :at-val:`"shell"` に設定することで指定できます。

.. _body-geom-solmix:

:at:`solmix`: :at-val:`real, "1"`
   この属性は、接触パラメータの平均化に使用される重みを指定し、priority属性と相互作用します。 :ref:`CContact` を参照してください。

.. _body-geom-solref:

.. _body-geom-solimp:

:at:`solref`, :at:`solimp`
   接触シミュレーション用の制約ソルバーパラメータです。 :ref:`CSolver` を参照してください。

.. _body-geom-margin:

:at:`margin`: :at-val:`real, "0"`
   接触が検出され、グローバル配列mjData.contactに含まれる距離閾値です。ただし、これは接触力が生成されることを意味するわけではありません。接触は、2つのジオム表面間の距離がmargin-gap未満の場合にのみアクティブと見なされます。制約インピーダンスは、 :ref:`CSolver` で説明されているように、距離の関数になり得ることを思い出してください。この関数が適用される量は、2つのジオム間の距離からマージンを引いてギャップを加えた値です。

.. _body-geom-gap:

:at:`gap`: :at-val:`real, "0"`
   この属性は、非アクティブな接触、つまり制約ソルバーによって無視されるが、カスタム計算の目的でmjData.contactに含まれる接触の生成を有効にするために使用されます。この値が正の場合、marginとmargin-gapの間のジオム距離は、このような非アクティブな接触に対応します。

.. _body-geom-fromto:

:at:`fromto`: :at-val:`real(6), optional`
   .. figure:: images/XMLreference/fromto.png
      :width: 350px
      :align: right

   この属性は、カプセル、ボックス、円筒、および楕円体ジオムでのみ使用できます。これは、ジオムの長さ、フレームの位置、および方向の代替仕様を提供します。6つの数値は、ある点の3D座標の後に別の点の3D座標が続きます。ジオムの細長い部分はこれらの2つの点を接続し、ジオムのフレームの+Z軸が最初の点から2番目の点に向かって方向付けられ、垂直方向では、ジオムのサイズは両方とも :at:`size` 属性の最初の値に等しくなります。フレームの方向は、 :ref:`Frame orientations <COrientation>` で説明されている :at:`zaxis` 属性と同じ手順で取得されます。フレームの位置は終点の中間にあります。この属性が指定されている場合、残りの位置と方向関連の属性は無視されます。右の画像は、同一のZ値を使用して、サポートされている4つのジオムでの :at:`fromto` の使用を示しています。モデルは `こちら <_static/fromto.xml>`__ です。 *カプセル* の :at:`fromto` セマンティクスは独特であることに注意してください：2つの終点は、半径がカプセル表面を定義するセグメントを指定します。

.. _body-geom-pos:

:at:`pos`: :at-val:`real(3), "0 0 0"`
   ジオムが定義されているボディのフレーム内のジオムの位置です。

.. _body-geom-quat:

.. _body-geom-axisangle:

.. _body-geom-xyaxes:

.. _body-geom-zaxis:

.. _body-geom-euler:

:at:`quat`, :at:`axisangle`, :at:`xyaxes`, :at:`zaxis`, :at:`euler`
   ジオムフレームの方向です。 :ref:`COrientation` を参照してください。

.. _body-geom-hfield:

:at:`hfield`: :at-val:`string, optional`
   この属性は、ジオムタイプが "hfield" の場合に指定する必要があります。ジオムフレームの位置と方向でインスタンス化されるハイトフィールドアセットを参照します。

.. _body-geom-mesh:

:at:`mesh`: :at-val:`string, optional`
   ジオムタイプが "mesh" の場合、この属性は必須です。インスタンス化されるメッシュアセットを参照します。この属性は、ジオムタイプが幾何学プリミティブ、つまり "sphere"、"capsule"、"cylinder"、"ellipsoid"、"box" のいずれかに対応する場合にも指定できます。その場合、プリミティブはここで参照されるメッシュアセットに自動的にフィットされます。フィッティング手順は、 :ref:`compiler <compiler>` のfitaabb属性によって決定される、メッシュの等価慣性ボックスまたは軸並行バウンディングボックスを使用します。フィットされたジオムの結果のサイズは通常期待されるものですが、そうでない場合は、以下のfitscale属性でさらに調整できます。コンパイルされたmjModelでは、ジオムは指定されたプリミティブタイプの通常のジオムとして表現され、フィッティングに使用されたメッシュへの参照はありません。

.. _body-geom-fitscale:

:at:`fitscale`: :at-val:`real, "1"`
   この属性は、プリミティブ幾何学タイプがメッシュアセットにフィットされている場合にのみ使用されます。ここで指定されたスケールは、自動フィッティング手順の出力に対する相対的なものです。デフォルト値の1は結果を変更せず、値2はフィットされたジオムのすべてのサイズを2倍にします。

.. _body-geom-fluidshape:

:at:`fluidshape`: :at-val:`[none, ellipsoid], "none"`
   "ellipsoid" は、ジオム形状の楕円体近似に基づくジオムレベルの流体相互作用モデルを有効にします。アクティブな場合、 :ref:`ボディ慣性サイズ <flInertia>` に基づくモデルは、ジオムが定義されているボディに対して無効になります。詳細については、 :ref:`楕円体ベース<flEllipsoid>` 流体相互作用モデルのセクションを参照してください。

.. _body-geom-fluidcoef:

:at:`fluidcoef`: :at-val:`real(5), "0.5 0.25 1.5 1.0 1.0"`
   流体相互作用モデルの無次元係数です。詳細については、 :ref:`楕円体ベース<flEllipsoid>` 流体相互作用モデルのセクションを参照してください。


.. list-table::
   :width: 60%
   :align: left
   :widths: 1 5 2 1
   :header-rows: 1

   * - インデックス
     - 説明
     - 記号
     - デフォルト
   * - 0
     - 鈍頭抗力係数
     - :math:`C_{D, \text{blunt}}`
     - 0.5
   * - 1
     - 細長抗力係数
     - :math:`C_{D, \text{slender}}`
     - 0.25
   * - 2
     - 角度抗力係数
     - :math:`C_{D, \text{angular}}`
     - 1.5
   * - 3
     - Kutta揚力係数
     - :math:`C_K`
     - 1.0
   * - 4
     - Magnus揚力係数
     - :math:`C_M`
     - 1.0


.. _body-geom-user:

:at:`user`: :at-val:`real(nuser_geom), "0 0 ..."`
   :ref:`CUser` を参照してください。

.. _geom-plugin:

:el-prefix:`geom/` |-| **plugin** |?|
'''''''''''''''''''''''''''''''''''''

このジオムを :ref:`エンジンプラグイン<exPlugin>` に関連付けます。 :at:`plugin` または :at:`instance` のいずれかが必要です。

.. _geom-plugin-plugin:

:at:`plugin`: :at-val:`string, optional`
   プラグイン識別子で、暗黙的なプラグインインスタンス化に使用されます。

.. _geom-plugin-instance:

:at:`instance`: :at-val:`string, optional`
   インスタンス名で、明示的なプラグインインスタンス化に使用されます。


.. _body-site:

:el-prefix:`body/` |-| **site** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

この要素はサイトを作成します。サイトはジオムの簡略化・制限版です。ジオム属性の小さなサブセットがここで利用できます。詳細なドキュメントについては :ref:`geom <body-geom>` 要素を参照してください。意味的にサイトは、ボディフレームに対する関心のある位置を表します。サイトは衝突やボディの質量と慣性の計算には参加しません。サイトのレンダリングに使用できる幾何学形状は、利用可能なジオムタイプのサブセットに限定されています。ただし、サイトは、ジオムが許可されていない場所で使用できます：センサーの取り付け、空間テンドンの経由点の指定、アクチュエータのスライダー・クランク伝達の構築などです。


.. _body-site-name:

:at:`name`: :at-val:`string, optional`
   サイトの名前です。

.. _body-site-class:

:at:`class`: :at-val:`string, optional`
   指定されていない属性を設定するためのデフォルトクラスです。

.. _body-site-type:

:at:`type`: :at-val:`[sphere, capsule, ellipsoid, cylinder, box], "sphere"`
   幾何学形状のタイプです。これはレンダリングに使用され、また :ref:`タッチセンサー <sensor-touch>` のアクティブセンサーゾーンも決定します。

.. _body-site-group:

:at:`group`: :at-val:`int, "0"`
   サイトが属する整数グループです。この属性はカスタムタグに使用できます。また、ビジュアライザーがサイトグループ全体のレンダリングを有効化・無効化するためにも使用されます。

.. _body-site-material:

:at:`material`: :at-val:`string, optional`
   サイトの視覚的プロパティを指定するために使用されるマテリアルです。

.. _body-site-rgba:

:at:`rgba`: :at-val:`real(4), "0.5 0.5 0.5 1"`
   色と透明度です。この値が内部デフォルトと異なる場合、対応するマテリアルプロパティを上書きします。

.. _body-site-size:

:at:`size`: :at-val:`real(3), "0.005 0.005 0.005"`
   サイトを表す幾何学形状のサイズです。

.. _body-site-fromto:

:at:`fromto`: :at-val:`real(6), optional`
   この属性は、カプセル、円筒、楕円体、およびボックスサイトでのみ使用できます。これは、サイトの長さ、フレームの位置、および方向の代替仕様を提供します。6つの数値は、ある点の3D座標の後に別の点の3D座標が続きます。サイトの細長い部分はこれらの2つの点を接続し、サイトのフレームの+Z軸が最初の点から2番目の点に向かって方向付けられます。フレームの方向は、 :ref:`Frame orientations <COrientation>` で説明されている zaxis 属性と同じ手順で取得されます。フレームの位置は2つの点の中間にあります。この属性が指定されている場合、残りの位置と方向関連の属性は無視されます。

.. _body-site-pos:

:at:`pos`: :at-val:`real(3), "0 0 0"`
   サイトフレームの位置です。

.. _body-site-quat:

.. _body-site-axisangle:

.. _body-site-xyaxes:

.. _body-site-zaxis:

.. _body-site-euler:

:at:`quat`, :at:`axisangle`, :at:`xyaxes`, :at:`zaxis`, :at:`euler`
   サイトフレームの方向です。 :ref:`COrientation` を参照してください。

.. _body-site-user:

:at:`user`: :at-val:`real(nuser_site), "0 0 ..."`
   :ref:`CUser` を参照してください。


.. _body-camera:

:el-prefix:`body/` |-| **camera** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

この要素はカメラを作成します。カメラは、定義されているボディとともに移動します。固定カメラを作成するには、ワールドボディで定義します。ここで作成されたカメラは、常に定義されており :ref:`visual <visual>` 要素で調整されるデフォルトのフリーカメラに追加されます。内部的にMuJoCoは柔軟なカメラモデルを使用し、仮想環境に必要な斜投影を得るために、視点と投影面が独立して調整されます。ただし、この機能はMJCFからはアクセスできません。代わりに、この要素で作成されたカメラ（およびフリーカメラ）は、常に投影面の前の中心に配置された視点を持ちます。視点はカメラフレームの中心と一致します。カメラはそのフレームの-Z軸に沿って見ます。+X軸は右を指し、+Y軸は上を指します。したがって、フレームの位置と方向がここで行う必要がある主要な調整です。


.. _body-camera-name:

:at:`name`: :at-val:`string, optional`
   カメラの名前です。

.. _body-camera-class:

:at:`class`: :at-val:`string, optional`
   指定されていない属性を設定するためのデフォルトクラスです。

.. _body-camera-mode:

:at:`mode`: :at-val:`[fixed, track, trackcom, targetbody, targetbodycom], "fixed"`
   この属性は、順運動学においてワールド座標でのカメラの位置と方向がどのように計算されるかを指定します（これがカメラが何を見るかを決定します）。 "fixed" は、以下で指定された位置と方向が、カメラが定義されているボディに対して固定されていることを意味します。 "track" は、カメラの位置がワールド座標でボディからの一定のオフセットにあり、カメラの方向がワールド座標で一定であることを意味します。これらの定数は、qpos0で順運動学を適用し、カメラを固定として扱うことによって決定されます。トラッキングは、例えばボディの上にカメラを配置し、下を向けてボディを見るようにし、ボディがどのように並進・回転してもカメラが常にボディの上に留まるようにするために使用できます。 "trackcom" は "track" に似ていますが、一定の空間オフセットは、カメラが定義されているボディから始まるキネマティックサブツリーの重心に対して定義されます。これはメカニズム全体を視野に保つために使用できます。ワールドボディのサブツリー重心はモデル全体の重心であることに注意してください。したがって、カメラがワールドボディに "trackcom" モードで定義されている場合、モデル全体を追跡します。 "targetbody" は、カメラの位置がボディフレームで固定されている一方、カメラの方向は常にターゲットボディ（以下のtarget属性で指定）に向けられるように調整されることを意味します。これは、例えば移動する物体を固視する目をモデル化するために使用できます。物体がターゲットになり、カメラ/目は頭に対応するボディで定義されます。 "targetbodycom" は "targetbody" と同じですが、カメラはターゲットボディから始まるサブツリーの重心に向けられます。

.. _body-camera-target:

:at:`target`: :at-val:`string, optional`
   カメラモードが "targetbody" または "targetbodycom" の場合、この属性は必須になります。カメラがターゲットにすべきボディを指定します。他のすべてのモードでは、この属性は無視されます。

.. _body-camera-projection:

:at:`projection`: :at-val:`[perspective, orthographic], "perspective"`
   カメラが透視投影（デフォルト）または正投影を使用するかどうかです。この属性を "orthographic" に設定すると、 :ref:`fovy<body-camera-fovy>` 属性のセマンティクスが変更されます。以下を参照してください。

.. _body-camera-fovy:

:at:`fovy`: :at-val:`real, "45"`
   カメラの垂直視野です。カメラが透視投影を使用する場合、視野はグローバルな :ref:`compiler/angle <compiler-angle>` 設定に関係なく、度で表現されます。カメラが正投影を使用する場合、視野は長さの単位で表現されます。この場合、デフォルトの45はほとんどのシーンで大きすぎるため、おそらく減らす必要があることに注意してください。いずれの場合も、水平視野はウィンドウサイズと垂直視野から自動的に計算されます。

.. _body-camera-resolution:

:at:`resolution`: :at-val:`int(2), "1 1"`
   カメラの解像度をピクセル単位で [幅 高さ] で指定します。これらの値はレンダリングには使用されないことに注意してください。レンダリングの寸法はレンダリングコンテキストのサイズによって決定されるためです。この属性は、必要な解像度を保存するための便利な場所として機能します。いずれかの値を1より大きく設定すると、 :ref:`mjVIS_CAMERA<mjtVisFlag>` 可視化フラグがアクティブな場合に視錐台の可視化が有効になります。

.. _body-camera-output:

:at:`output`: :at-val:`[rgb, depth, distance, normal, segmentation], "rgb"`
   カメラがサポートする出力画像のタイプです。

   - :at-val:`rgb`: RGB画像。
   - :at-val:`depth`: 深度画像（カメラ平面からの距離）。
   - :at-val:`distance`: 距離画像（カメラ原点からの距離）。
   - :at-val:`normal`: 表面法線画像。
   - :at-val:`segmentation`: セグメンテーション画像。

   この属性はレンダリングには使用されませんが、カメラがサポートする出力タイプを保存するための便利な場所として機能します。 :at:`output` 属性には複数のタイプを含めることができます。例えば :at-val:`"rgb normal"` のようにします。

.. _body-camera-sensorsize:

:at:`sensorsize`: :at-val:`real(2), "0 0"`
   カメラセンサーのサイズを長さ単位で指定します。指定されると、すべての内部属性がアクティブになり、 :at:`fovy` は無視されます。視野は焦点距離とセンサーサイズから自動的に計算されます。

.. _body-camera-focal:
.. _body-camera-focalpixel:

:at:`focal` / :at:`focalpixel`: :at-val:`real(2), "0 0"`
   物理的な長さ単位またはピクセル単位での焦点距離です。両方が指定されている場合、ピクセル値が使用され、長さ値は無視されます。

.. _body-camera-principal:
.. _body-camera-principalpixel:

:at:`principal` / :at:`principalpixel`: :at-val:`real(2), "0 0"`
   画像中心からの主点（光軸と画像平面の交点）のオフセットです。両方が指定されている場合、ピクセル値が使用されます。オフセットがゼロの場合、レンダリングされた画像はカメラの負のZ軸を中心にしており、標準的なピンホールカメラモデルと同様です。

.. _body-camera-ipd:

:at:`ipd`: :at-val:`real, "0.068"`
   瞳孔間距離です。この属性は立体視レンダリング中にのみ効果があります。左右の視点間の距離を指定します。各視点は、カメラフレームのX軸に沿って、ここで指定された距離の半分だけ +/- シフトされます。

.. _body-camera-pos:

:at:`pos`: :at-val:`real(3), "0 0 0"`
   カメラフレームの位置です。

.. _body-camera-quat:

.. _body-camera-axisangle:

.. _body-camera-xyaxes:

.. _body-camera-zaxis:

.. _body-camera-euler:

:at:`quat`, :at:`axisangle`, :at:`xyaxes`, :at:`zaxis`, :at:`euler`
   カメラフレームの方向です。 :ref:`COrientation` を参照してください。特にカメラの場合、 :at:`xyaxes` 属性は意味的に便利です。X軸とY軸がピクセル空間でそれぞれ「右」と「上」の方向に対応するためです。

.. _body-camera-user:

:at:`user`: :at-val:`real(nuser_cam), "0 0 ..."`
   :ref:`CUser` を参照してください。


.. _body-light:

:el-prefix:`body/` |-| **light** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

この要素はライトを作成します。ライトは、定義されているボディとともに移動します。固定ライトを作成するには、ワールドボディで定義します。ここで作成されたライトは、常に定義されており :ref:`visual <visual>` 要素で設定されるヘッドライトに追加されます。ライトは、dir属性で指定された方向に沿って照らします。3つの直交軸を持つ完全な空間フレームはありません。

デフォルトでは、MuJoCoはレンダリングに標準的なOpenGL（固定機能）Phongライティングモデルを使用し、シャドウマッピングで拡張されています（詳細については、さまざまな属性に関する詳細を含め、OpenGLドキュメントを参照してください）。

MJCFは、追加の属性を提供することで、代替のライティングモデル（例：物理ベースレンダリング）もサポートします。属性は、使用されているライティングモデルに応じて適用または無視される場合があります。


.. _body-light-name:

:at:`name`: :at-val:`string, optional`
   ライトの名前です。

.. _body-light-class:

:at:`class`: :at-val:`string, optional`
   指定されていない属性を設定するためのデフォルトクラスです。

.. _body-light-mode:

:at:`mode`: :at-val:`[fixed, track, trackcom, targetbody, targetbodycom], "fixed"`
   これは上記の :ref:`camera <body-camera>` のmode属性と同じです。順運動学においてワールド座標でのライトの位置と方向がどのように計算されるかを指定します（これがライトが何を照らすかを決定します）。

.. _body-light-target:

:at:`target`: :at-val:`string, optional`
   これは上記の :ref:`camera <body-camera>` のtarget属性と同じです。 "targetbody" および "targetbodycom" モードでターゲットにすべきボディを指定します。

.. _body-light-type:

:at:`type`: :at-val:`[spot, directional, point, image], "spot"`
   ライトのタイプを決定します。一部のライトタイプは一部のレンダラーでサポートされていない場合があることに注意してください（例えば、デフォルトのネイティブレンダラーではスポットライトとディレクショナルライトのみがサポートされています）。

.. _body-light-directional:

:at:`directional`: :at-val:`[false, true], "false"`
   これは非推奨のレガシー属性です。代わりに :ref:`light <body-light-type>` タイプを使用してください。 "true" に設定され、タイプが指定されていない場合、ライトタイプはディレクショナルに変更されます。

.. _body-light-castshadow:

:at:`castshadow`: :at-val:`[false, true], "true"`
   この属性が "true" の場合、ライトは影を落とします。より正確には、ライトによって照らされるジオムが影を落としますが、これはジオムではなくライトの特性です。各影を落とすライトは、すべてのジオムを通る追加のレンダリングパスを引き起こすため、この属性は注意して使用する必要があります。影の高品質は、 :ref:`quality <visual-quality>` のshadowsize属性の値を増やすこと、スポットライトを影が現れる表面に近づけること、および影が落とされる体積を制限することによって達成されます。スポットライトの場合、この体積は円錐であり、その角度は以下のcutoff属性に :ref:`map <visual-map>` のshadowscale属性を掛けたものです。ディレクショナルライトの場合、この体積はボックスであり、光に直交する方向のハーフサイズは、 :ref:`map <visual-map>` のshadowclip属性を掛けたモデルエクステントです。モデルエクステントはコンパイラによって計算されますが、 :ref:`statistic <statistic>` のextent属性を指定することで上書きすることもできます。内部的に、シャドウマッピングメカニズムは、ライトの視点（カメラであるかのように）からシーンを深度テクスチャにレンダリングし、次にカメラの視点から再度レンダリングし、深度テクスチャを使用して影を作成します。内部レンダリングパスは、通常のレンダリングと同じ近距離および遠距離クリッピング面を使用します。つまり、これらのクリッピング面は、光方向の円錐またはボックス影体積を制限します。その結果、一部の影（特にライトに非常に近い影）がクリップされる場合があります。

.. _body-light-active:

:at:`active`: :at-val:`[false, true], "true"`
   この属性が "true" の場合、ライトはアクティブです。これは実行時にライトをオン・オフするために使用できます。

.. _body-light-pos:

:at:`pos`: :at-val:`real(3), "0 0 0"`
   ライトの位置です。この属性はスポットライトのレンダリングにのみ影響しますが、カメラを装飾要素としてレンダリングするため、ディレクショナルライトに対しても定義する必要があります。

.. _body-light-dir:

:at:`dir`: :at-val:`real(3), "0 0 -1"`
   ライトの方向です。

.. _body-light-diffuse:

:at:`diffuse`: :at-val:`real(3), "0.7 0.7 0.7"`
   ライトの色です。Phong（デフォルト）ライティングモデルの場合、これはライトの拡散色を定義します。

.. _body-light-texture:

:at:`texture`: :at-val:`string, optional`
   画像ベースのライティングに使用するテクスチャです。これはデフォルトのPhongライティングモデルでは使用されません。

.. _body-light-intensity:

:at:`intensity`: :at-val:`real, "0.0"`
   物理ベースのライティングモデルに使用される、カンデラ単位で測定されるライトソースの強度です。これはデフォルトのPhongライティングモデルでは使用されません。

.. _body-light-ambient:

:at:`ambient`: :at-val:`real(3), "0 0 0"`
   デフォルトのPhongライティングモデルで使用される、ライトの環境色です。

.. _body-light-specular:

:at:`specular`: :at-val:`real(3), "0.3 0.3 0.3"`
   デフォルトのPhongライティングモデルで使用される、ライトの鏡面色です。

.. _body-light-range:

:at:`range`: :at-val:`real, "10.0"`
   ライトの有効範囲です。ライトの位置からこの距離よりも遠い物体は、このライトによって照らされません。これはスポットライトにのみ適用されます。

.. _body-light-bulbradius:

:at:`bulbradius`: :at-val:`real, "0.02"`
   レンダラーに応じて影の柔らかさに影響を与える可能性のある光源の半径です。これはスポットライトにのみ適用されます。

.. _body-light-attenuation:

:at:`attenuation`: :at-val:`real(3), "1 0 0"`
   これらは、Phongライティングの定数、線形、および二次減衰係数です。デフォルトは減衰なしに対応します。

.. _body-light-cutoff:

:at:`cutoff`: :at-val:`real, "45"`
   スポットライトのカットオフ角度で、グローバルな角度設定に関係なく常に度で表されます。

.. _body-light-exponent:

:at:`exponent`: :at-val:`real, "10"`
   スポットライトの指数です。この設定はスポットライトのカットオフの柔らかさを制御します。

.. _body-composite:

:el-prefix:`body/` |-| **composite** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

これはモデル要素ではなく、複合オブジェクトを表す複数のモデル要素に展開されるマクロです。これらの要素は、マクロを含む親ボディの子となるボディ（独自のジョイントとジオムを持つ）です。マクロ展開は、モデルコンパイラによって行われます。結果として得られたモデルを保存すると、マクロは実際のモデル要素に置き換えられます。MJCF の他の部分で使用されるデフォルトメカニズムは、親ボディに childclass 属性が定義されている場合でも、ここでは適用されません。代わりに、各複合オブジェクトタイプに対して自動的に調整される内部デフォルトがあります。より詳細な説明については、モデリングガイドの :ref:`CComposite` を参照してください。以前はいくつかの複合タイプがありましたが、それらは徐々に :ref:`replicate<replicate>` （繰り返しオブジェクト用）と :ref:`flexcomp<body-flexcomp>` （ソフトオブジェクト用）に置き換えられてきました。したがって、現在サポートされている複合タイプは cable のみで、これはボールジョイントで接続された伸縮不可能なボディのチェーンを生成します。

.. _body-composite-prefix:

:at:`prefix`: :at-val:`string, optional`
   自動的に生成されるすべてのモデル要素には、要素タイプとインデックスを示す名前が付けられます。たとえば、2D グリッドの座標 (2, 0) にあるボディは、デフォルトで "B2_0" という名前になります。prefix="C" が指定されると、同じボディは "CB2_0" という名前になります。prefix は、名前の競合を避けるために、同じモデルで複数の複合オブジェクトを使用する場合に必要です。

.. _body-composite-type:

:at:`type`: :at-val:`[cable], required`
   この属性は、複合オブジェクトのタイプを決定します。サポートされているタイプは cable のみです。

   **cable** タイプは、ボールジョイントで接続された 1D のボディチェーンを作成し、各ボディはユーザー定義のタイプ（円筒、カプセル、またはボックス）のジオムを持ちます。ジオメトリは、3D 頂点座標の配列 :at:`vertex` で定義するか、 :at:`curve` オプションを使用して規定された関数で定義できます。現在、線形関数と三角関数のみがサポートされています。たとえば、螺旋は curve="cos(s) sin(s) s" で得られます。サイズは :at:`size` オプションで設定され、 :math:`f(s)=\{\text{size}[1]\cdot\cos(2\pi\cdot\text{size}[2]),\;
   \text{size}[1]\cdot\sin(2\pi\cdot\text{size}[2]),\; \text{size}[0]\cdot s\}` となります。

.. _body-composite-count:

:at:`count`: :at-val:`int(3), required`
   グリッドの各次元の要素数です。これは、1、2、または 3 つの数値を持つことができ、親ボディフレーム内の X、Y、Z 軸に沿った要素数を指定します。欠落している数値はデフォルトで 1 になります。これらの数値のいずれかが 1 の場合、それ以降のすべての数値も 1 でなければならないため、グリッドの先頭の次元が使用されます。これは、たとえば 1D グリッドが常に X 軸に沿って伸びることを意味します。異なる向きを実現するには、親ボディのフレームを回転させます。一部のタイプは特定の次元のグリッドを意味するため、この属性の要件は指定されたタイプに依存します。

.. _body-composite-offset:

:at:`offset`: :at-val:`real(3), "0 0 0"`
   親ボディの中心からケーブルの最初のボディの中心までの 3D オフセットを指定します。オフセットは、親ボディのローカル座標フレームで表現されます。

.. _body-composite-quat:

:at:`quat`: :at-val:`real(4), "1 0 0 0"`
   最初のボディフレームを回転させるクォータニオンを指定します。クォータニオンは、親ボディフレームで表現されます。

.. _body-composite-vertex:

:at:`vertex`: :at-val:`real(3*nvert), optional`
   グローバル座標での頂点の 3D 位置です。

.. _body-composite-initial:

:at:`initial`: :at-val:`[free, ball, none], "0"`
   最初の点の動作です。Free: フリージョイント。Ball: ボールジョイント。None: 自由度なし。

.. _body-composite-curve:

:at:`curve`: :at-val:`string(3), optional`
   頂点位置を指定する関数です。使用可能な関数は `s` 、 `cos(s)` 、および `sin(s)` で、 `s` は弧長パラメータです。

.. _body-composite-size:

:at:`size`: :at-val:`int(3), optional`
   曲線関数のスケーリングです。 `size[0]` は `s` のスケーリング、 `size[1]` は `\cos(s)` と `\sin(s)` の半径、 `size[2]` は引数の速度（つまり `\cos(2*\pi*size[2]*s)` ）です。


.. _composite-joint:

:el-prefix:`composite/` |-| **joint** |m|
'''''''''''''''''''''''''''''''''''''''''

複合タイプに応じて、一部のジョイントは自動的に作成されます（たとえば、rope のユニバーサルジョイント）が、他のジョイントはオプションです（たとえば、rope のストレッチジョイントとツイストジョイント）。このサブ要素は、どのオプションジョイントを作成するかを指定し、自動ジョイントとオプションジョイントの両方の属性を調整するために使用されます。

.. _composite-joint-kind:

:at:`kind`: :at-val:`[main], required`
   ここでのジョイント種別（kind）は、MJCF の他の部分でのジョイントタイプ（type）とは直交する概念です。ジョイント種別は、複合ボディを構成するメカニズム内でのジョイントの機能を指し、一方、ジョイントタイプ（ヒンジまたはスライド）は、ジョイント種別と複合ボディタイプによって暗黙的に決まります。

   **main** 種別は、各複合タイプを形成する主要なジョイントに対応します。これらのジョイントは、joint サブ要素が欠落している場合でも、自動的にモデルに含まれます。主要なジョイントは、particle と grid の場合は 3D スライダー、box、cylinder、rope の場合は 1D スライダー、cloth、rope、loop の場合はユニバーサルジョイントです。主要なジョイントは自動的に含まれますが、このサブ要素はそれらの属性を調整するのに依然として役立ちます。

.. _composite-joint-solreffix:

.. _composite-joint-solimpfix:

:at:`solreffix`, :at:`solimpfix`
   これらは、ジョイントを等式制約するために使用される solref および solimp 属性です。特定のジョイントが等式制約されているかどうかは、上記で説明したジョイント種別と複合オブジェクトタイプに依存します。等式制約されていないジョイントの場合、この属性は効果がありません。デフォルトは複合タイプに応じて調整されます。それ以外の場合、これらの属性は MJCF の他のすべての solref および solimp 属性と同じルールに従います。 :ref:`Solver parameters <CSolver>` を参照してください。

.. _composite-joint-axis:

.. _composite-joint-group:

.. _composite-joint-stiffness:

.. _composite-joint-damping:

.. _composite-joint-armature:

.. _composite-joint-limited:

.. _composite-joint-range:

.. _composite-joint-margin:

.. _composite-joint-solreflimit:

.. _composite-joint-solimplimit:

.. _composite-joint-frictionloss:

.. _composite-joint-solreffriction:

.. _composite-joint-solimpfriction:

.. _composite-joint-type:

.. |body/composite/joint attrib list| replace::
   :at:`axis`, :at:`group`, :at:`stiffness`, :at:`damping`, :at:`armature`, :at:`limited`, :at:`range`, :at:`margin`,
   :at:`solreflimit`, :at:`solimplimit`, :at:`frictionloss`, :at:`solreffriction`, :at:`solimpfriction`, :at:`type`

|body/composite/joint attrib list|
   通常の :ref:`joint <body-joint>` 属性と同じ意味です。


.. _composite-geom:

:el-prefix:`composite/` |-| **geom** |?|
''''''''''''''''''''''''''''''''''''''''

このサブ要素は、複合オブジェクト内のジオムの属性を調整します。デフォルト属性は、MJCF の他の部分と同じです（ただし、ユーザー定義のデフォルトはここでは効果がありません）。geom サブ要素は一度しか出現できないことに注意してください。これは、異なる種別の joint や tendon のサブ要素が複数回出現できるのとは異なります。これは、異なる種別のジョイントとテンドンは異なる属性セットを持つのに対し、複合オブジェクト内のすべてのジオムは同一だからです。

.. _composite-geom-type:

.. _composite-geom-contype:

.. _composite-geom-conaffinity:

.. _composite-geom-condim:

.. _composite-geom-group:

.. _composite-geom-priority:

.. _composite-geom-size:

.. _composite-geom-material:

.. _composite-geom-rgba:

.. _composite-geom-friction:

.. _composite-geom-mass:

.. _composite-geom-density:

.. _composite-geom-solmix:

.. _composite-geom-solref:

.. _composite-geom-solimp:

.. _composite-geom-margin:

.. _composite-geom-gap:

.. |body/composite/geom attrib list| replace::
   :at:`type`, :at:`contype`, :at:`conaffinity`, :at:`condim`, :at:`group`, :at:`priority`, :at:`size`, :at:`material`,
   :at:`rgba`, :at:`friction`, :at:`mass`, :at:`density`, :at:`solmix`, :at:`solref`, :at:`solimp`, :at:`margin`,
   :at:`gap`

|body/composite/geom attrib list|
   通常の :ref:`geom <body-geom>` 属性と同じ意味です。


.. _composite-site:

:el-prefix:`composite/` |-| **site** |?|
''''''''''''''''''''''''''''''''''''''''

このサブ要素は、複合オブジェクト内のサイトの属性を調整します。それ以外は上記の geom と同じです。

.. _composite-site-group:

.. _composite-site-size:

.. _composite-site-material:

.. _composite-site-rgba:

:at:`group`, :at:`size`, :at:`material`, :at:`rgba`
   通常の :ref:`site <body-site>` 属性と同じ意味です。


.. _composite-skin:

:el-prefix:`composite/` |-| **skin** |?|
''''''''''''''''''''''''''''''''''''''''

この要素が含まれている場合、モデルコンパイラは、スキンメッシュアセットを生成し、それを複合オブジェクトの要素ボディにアタッチします。スキンは、2D グリッド、cloth、box、cylinder、ellipsoid にアタッチできます。他の複合タイプには効果がありません。ここで作成されるスキンは、ファイルから読み込まれるスキンではなく、XML で直接指定されるスキンと同等であることに注意してください。したがって、モデルを XML として保存すると、自動生成されたスキンを記述する大きなセクションが含まれます。

.. _composite-skin-texcoord:

:at:`texcoord`: :at-val:`[false, true], "false"`
   これが true の場合、明示的なテクスチャ座標が生成され、スキンをテクスチャ空間の単位正方形にマッピングします。これは、マテリアルがテクスチャを指定する場合に必要です。texcoord が false でスキンにテクスチャがある場合、テクスチャはスキンではなく世界に固定されているように見えます。そもそもこの属性がある理由は、テクスチャ座標を持つスキンは、後でテクスチャが適用されない場合でも、これらの座標を GPU にアップロードするからです。したがって、material 属性を介してテクスチャが適用されない場合は、この属性を false に設定する必要があります。

.. _composite-skin-material:

.. _composite-skin-rgba:

.. _composite-skin-group:

:at:`material`, :at:`rgba`, :at:`group`:
   :ref:`geom <body-geom>` と同じ意味です。

.. _composite-skin-inflate:

:at:`inflate`: :at-val:`real, "0"`
   デフォルト値の 0 は、自動生成されたスキンが、複合オブジェクトを構成するボディ要素の中心を通過することを意味します。正の値は、各スキン頂点を、その頂点における（膨張していない）スキンの法線方向に、指定された量だけオフセットします。これには 2 つの用途があります。第一に、2D オブジェクトでは、エイリアシングアーティファクトを避けるために、小さな正の膨張係数が必要です。第二に、衝突は厚みを作り出すジオムで行われます。これは 2D オブジェクトの場合でも同様です。ジオムサイズに等しい値でスキンを膨張させると、実際の衝突ジオメトリをよりよく表す「マットレス」としてスキンがレンダリングされます。この属性の値は、作成される :el:`skin` アセットの対応する属性にコピーされます。

.. _composite-skin-subgrid:

:at:`subgrid`: :at-val:`int, "0"`
   これは cloth と 2D grid タイプにのみ適用され、他の複合タイプには効果がありません。デフォルト値の 0 は、スキンが要素ボディの数と同じ数の頂点を持つことを意味します。正の値は細分化を引き起こし、指定された数の（追加の）グリッド線を持ちます。この場合、モデルコンパイラは、双三次補間を使用して、より密なスキンを生成します。これにより、レンダリングの品質が向上します（特にテクスチャがない場合）が、レンダラーも遅くなるため、注意して使用してください。3 を超える値はほとんど必要ありません。


.. _composite-plugin:

:el-prefix:`composite/` |-| **plugin** |?|
''''''''''''''''''''''''''''''''''''''''''

この composite を :ref:`engine plugin<exPlugin>` に関連付けます。 :at:`plugin` または :at:`instance` のいずれかが必要です。

.. _composite-plugin-plugin:

:at:`plugin`: :at-val:`string, optional`
   プラグイン識別子。暗黙的なプラグインインスタンス化に使用されます。

.. _composite-plugin-instance:

:at:`instance`: :at-val:`string, optional`
   インスタンス名。明示的なプラグインインスタンス化に使用されます。



.. _body-flexcomp:

:el-prefix:`body/` |-| **flexcomp** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

:el:`composite` と同様に、この要素（MuJoCo 3.0 で新規追加）はモデル要素ではなく、変形可能エンティティを表す複数のモデル要素に展開されるマクロです。特に、このマクロは、1 つの :ref:`flex<deformable-flex>` 要素、 :el:`flexcomp` が定義されているボディの子である多数のボディ、およびオプションで、すべてのフレックスエッジを初期長さに制約する 1 つの :ref:`flex equality<equality-flex>` を作成します。多くの属性がここで指定され、その後、自動構築されるフレックスに渡されます。 :el:`flexcomp` の主な役割は、対応するジョイントを持つ（場合によっては大規模な）移動ボディのコレクションの作成を自動化し、それらを伸縮可能なフレックス要素で接続することです。フレックスの動作の詳細については、 :ref:`flex<deformable-flex>` および :ref:`deformable objects<CDeformable>` のドキュメントを参照してください。ここでは、自動構築プロセスのみを説明します。

:el:`flex` と :el:`flexcomp` の重要な違いは、flex がボディを参照し、それらのボディのフレームで頂点座標を指定するのに対し、flexcomp は *点* を定義することです。各 flexcomp 点は、基礎となるフレックスの 1 つのボディと 1 つの頂点に対応します。flexcomp 点が *固定* されている場合、対応するフレックスボディは flexcomp の親ボディであり、対応するフレックス頂点座標は flexcomp 点座標に等しくなります。flexcomp 点が固定されていない場合、flexcomp 点の座標に新しい子ボディが（flexcomp 親ボディ内に）作成され、その新しいボディ内のフレックス頂点の座標は (0,0,0) になります。flexcomp 点を :ref:`固定<flexcomp-pin>` するメカニズムについては、以下で説明します。

複合オブジェクト（MuJoCo 3.0 より前に利用可能）は、衝突のためにジオムを持つボディ、およびテンドンを接続するためのサイトが必要でした。これらのテンドンは形状保存力を生成していました。対照的に、フレックスは独自の衝突と形状保存力（およびレンダリング）を生成するため、ここで作成されるボディははるかにシンプルです：ジオム、サイト、テンドンは必要ありません。ここで作成されるボディのほとんどは、自由に移動する点質量に対応する 3 つの直交スライダージョイントを持ちます。場合によっては、拡張と収縮のみを許可する半径方向スライダージョイントを生成します。ジオムが生成されないため、ボディには明示的な慣性パラメータが必要です。

以下は、一端が世界に固定された flexcomp の単純な例で、（やや柔軟な）二重振り子をモデル化しています：

.. code-block:: xml

   <mujoco>
     <worldbody>
       <flexcomp name="FL" type="grid" dim="1" count="3 1 1" mass="3" spacing="0.2 0.2 0.2">
         <pin id="0"/>
       </flexcomp>
     </worldbody>
   </mujoco>

この flexcomp には 3 つの点がありますが、最初の点は世界（つまり flexcomp の親）に固定されているため、FL_1 と FL_2 という 2 つのボディのみが自動的に作成されます。以下は、この flexcomp が XML をロードして保存した後に生成するものです：

.. code-block:: xml

   <mujoco>
     <worldbody>
       <body name="FL_1">
         <inertial pos="0 0 0" mass="1" diaginertia="1.66667e-05 1.66667e-05 1.66667e-05"/>
         <joint pos="0 0 0" axis="1 0 0" type="slide"/>
         <joint pos="0 0 0" axis="0 1 0" type="slide"/>
         <joint pos="0 0 0" axis="0 0 1" type="slide"/>
       </body>
       <body name="FL_2" pos="0.2 0 0">
         <inertial pos="0 0 0" mass="1" diaginertia="1.66667e-05 1.66667e-05 1.66667e-05"/>
         <joint pos="0 0 0" axis="1 0 0" type="slide"/>
         <joint pos="0 0 0" axis="0 1 0" type="slide"/>
         <joint pos="0 0 0" axis="0 0 1" type="slide"/>
       </body>
     </worldbody>
     <deformable>
       <flex name="FL" dim="1" body="world FL_1 FL_2" vertex="-0.2 0 0 0 0 0 0 0 0" element="0 1 1 2"/>
     </deformable>
     <equality>
       <flex flex="FL"/>
     </equality>
   </mujoco>


.. _body-flexcomp-name:

:at:`name`: :at-val:`string, required`
   自動生成される flex 要素の名前です。この名前は、ここで自動生成されるすべてのボディのプレフィックスとして使用され、対応する flex 等式制約（該当する場合）によっても参照されます。

.. _body-flexcomp-dim:

:at:`dim`: :at-val:`int(1), "2"`
   フレックスオブジェクトの次元です。この値は 1、2、または 3 でなければなりません。フレックス要素は、1D ではカプセル、2D では半径を持つ三角形、3D では半径を持つ四面体です。特定の flexcomp タイプは次元を暗黙的に指定するため、その場合、ここで指定された値は無視されます。

.. youtube:: uNt3i8hrJu4
   :align: right
   :width: 240px

.. _body-flexcomp-dof:

:at:`dof`: :at-val:`[full, radial, trilinear], "full"`
   フレックスの自由度（dofs）のパラメータ化です。右のビデオは、変形可能な球体を使用して異なるパラメータ化を示しています。ビデオの 3 つのモデルはそれぞれ
   `sphere_full <https://github.com/google-deepmind/mujoco/blob/main/model/flex/sphere_full.xml>`__ 、
   `sphere_radial <https://github.com/google-deepmind/mujoco/blob/main/model/flex/sphere_radial.xml>`__ 、
   `sphere_trilinear <https://github.com/google-deepmind/mujoco/blob/main/model/flex/sphere_trilinear.xml>`__ です。

   **full**
     頂点ごとに 3 つの並進自由度です。これは最も表現力が高いオプションですが、最も高コストでもあります。

   **radial**
     頂点ごとに 1 つの半径方向並進自由度です。"full" の場合とは異なり、半径方向パラメータ化では、自由な物体運動を可能にするために、フレックスの親にフリージョイントが必要であることに注意してください。このタイプのパラメータ化は、比較的球形の形状に適しています。

   **trilinear**
     フレックスのバウンディングボックスの各隅に 3 つの並進自由度があり、フレックス全体で合計 24 自由度になります。これは頂点数に依存しません。頂点の位置は、バウンディングボックス上の三線形補間を使用して更新されます。

   .. youtube:: qJFbx-FR7Bc
      :align: right
      :width: 240px

   三線形フレックスと二次フレックスは、前の 2 つのオプションよりもはるかに高速であり、期待される変形が縮約パラメータ化でキャプチャできる場合は、推奨される選択肢です。たとえば、右のビデオは、変形可能なグリッパーパッドをモデル化するための `full <https://github.com/google-deepmind/mujoco/blob/main/model/flex/gripper.xml>`__ フレックスと `trilinear
   <https://github.com/google-deepmind/mujoco/blob/main/model/flex/gripper_trilinear.xml>`__ フレックスを比較しています。

   dof パラメータ化の選択は、フレックスの変形モードに影響を与えますが、衝突ジオメトリの精度には影響を与えません。衝突ジオメトリは、常にフレックスの高解像度メッシュを考慮します。

   **quadratic**
     フレックスのバウンディングボックスの各隅、エッジ、面、体積に 3 つの並進自由度があり、フレックス全体で合計 81 自由度になります。これは頂点数に依存しません。頂点の位置は、バウンディングボックス上の二次補間を使用して更新されます。このオプションは三線形フレックスよりも多くの自由度を必要としますが、曲線変形モードを可能にします。一方、三線形フレックスで達成可能なモードは、伸縮/圧縮とせん断のみです。

   一般に、より高い補間次数では、安定性のためにより小さなタイムステップが必要ですが、通常は "full" オプションと細かいメッシュを使用する場合ほど大きくはありません。

.. _body-flexcomp-type:

:at:`type`: :at-val:`[grid, box, cylinder, ellipsoid, disc, circle, mesh, gmsh, direct], "grid"`
   この属性は、 :el:`flexcomp` オブジェクトのタイプを決定します。残りの属性とサブ要素は、タイプに応じて解釈されます。デフォルト設定もタイプに応じて調整されます。異なるタイプは、flexcomp 点とそれらを接続する伸縮可能要素を指定するための異なる方法に対応します。それらは 3 つのカテゴリに分類されます：XML で入力される直接指定、ファイルから読み込まれる直接指定、より高レベルの仕様からの自動生成です。

   **grid** は、 :at:`dim` で指定されるように、1D、2D、または 3D の長方形の点グリッドを生成します。各次元の点の数は :at:`count` で決定され、各次元のグリッド間隔は :at:`spacing` で決定されます。永続的な接触を避けるために、間隔が :at:`radius` に対して十分に大きいことを確認してください。2D と 3D では、グリッドは自動的に三角形分割され、対応するフレックス要素（三角形または四面体）が作成されます。1D では、要素は連続する点のペアを接続するカプセルです。

   **box** は 3D ボックスオブジェクトを生成しますが、フレックスボディは外側のシェルにのみ生成されます。各フレックスボディには、ボックスの中心から内外に移動できる半径方向スライダージョイントがあります。親ボディは通常、浮遊ボディになります。ボックス表面は三角形分割され、各フレックス要素は、ボックスの中心を 1 つの三角形面と接続する四面体です。 :at:`count` と :at:`spacing` は、3D の **grid** タイプと同様に、フレックスボディの数と間隔を決定します。結果として得られるフレックスは、 :el:`composite` によって生成されるボックスと同じトポロジーを持つことに注意してください。

   **cylinder** は **box** と同じですが、点が円筒の表面に投影されます。

   **ellipsoid** は **box** と同じですが、点が楕円体の表面に投影されます。

   **disc** は **box** と同じですが、点が円盤の表面に投影されます。 :at:`dim=2` とのみ互換性があります。

   **circle** は **grid** と同じですが、点が円に沿ってサンプリングされるため、最初と最後の点が同じになります。円の半径は、各セグメントが要求された間隔を持つように計算されます。 :at:`dim=1` とのみ互換性があります。

   **mesh** は、flexcomp 点と要素（つまり三角形）をメッシュファイルから読み込みます。メッシュアセットと同じファイル形式で、レガシーの .msh 形式は除外されます。メッシュアセットは実際にはモデルに追加されません。代わりに、メッシュファイルからの頂点と面のデータが、flexcomp の点と要素のデータを埋めるために使用されます。 :at:`dim` は自動的に 2 に設定されます。MuJoCo のメッシュアセットは、単一のボディにアタッチされた剛体ジオムとして使用できることを思い出してください。対照的に、ここで生成されるフレックスは、同じ初期形状を持つソフトメッシュに対応し、各頂点は個別の移動ボディです（固定されていない場合）。

   .. _gmsh-file-docs:

   **gmsh** は mesh と似ていますが、
   `format 4.1 <https://gmsh.info//doc/texinfo/gmsh.html#MSH-file-format>`__
   および `format 2.2 <https://gmsh.info//doc/texinfo/gmsh.html#MSH-file-format-version-2-_0028Legacy_0029>`__
   （ascii または binary）の GMSH ファイルを読み込みます。ファイル拡張子は任意で、パーサーはファイルヘッダーを調べることでフォーマットを認識します。これは非常にリッチなファイル形式で、異なる次元とトポロジーを持つあらゆる種類の要素を許可します。MuJoCo は GMSH 要素タイプ 1、2、4 のみをサポートしており、これらはたまたま 1D、2D、3D のフレックスに対応します。また、ノードが単一のブロックで指定されていることを前提としています。GMHS ファイルの Nodes セクションと Elements セクションのみが処理され、flexcomp の点と要素のデータを埋めるために使用されます。パーサーは、MuJoCo でサポートされていないメッシュが GMSH ファイルに含まれている場合、エラーを生成します。 :at:`dim` は GMSH ファイルで指定された次元に自動的に設定されます。現在、これは MuJoCo で大規模な四面体メッシュを読み込み、対応するソフトエンティティを生成する唯一のメカニズムです。そのようなメッシュが別のファイル形式で利用可能な場合、自由に利用可能な `GMSH software <https://gmsh.info/>`__ を使用して、サポートされているバージョンのいずれかで GMSH に変換してください。

   **direct** は、ユーザーが flexcomp の点と要素のデータを XML で直接指定できるようにします。flexcomp は依然として移動ボディを自動的に生成し、他の設定を自動化するため、対応するフレックスを直接指定するよりも利便性を提供します。

.. _body-flexcomp-count:

:at:`count`: :at-val:`int(3), "10 10 10"`
   各次元で自動生成される点の数です。この属性と次の属性は、grid、box、cylinder、ellipsoid タイプにのみ適用されます。

.. _body-flexcomp-spacing:

:at:`spacing`: :at-val:`real(3), "0.02 0.02 0.02"`
   各次元で自動生成される点間の間隔です。間隔は、永続的な接触を避けるために、半径に比べて十分に大きくする必要があります。

.. _body-flexcomp-point:

:at:`point`: :at-val:`real(3*npoint), optional`
   点の 3D 座標です。この属性は、type **direct** でのみ使用されます。他のすべての flexcomp タイプは、独自の点を生成します。点は、前述のようにボディと頂点を構築するために使用されます。

.. _body-flexcomp-element:

:at:`element`: :at-val:`int((dim+1)*npoint), optional`
   各フレックス要素を形成する 0 ベースの点 ID です。この属性は、type **direct** でのみ使用されます。他のすべての flexcomp タイプは、独自の要素を生成します。このデータは、自動生成されるフレックスに渡されます。

.. _body-flexcomp-texcoord:

:at:`texcoord`: :at-val:`real(2*npoint), optional`
   各点のテクスチャ座標で、自動生成されるフレックスに渡されます。flexcomp は、2D グリッド、box、cylinder、ellipsoid を除いて、テクスチャ座標を自動的に生成しないことに注意してください。他のすべてのタイプの場合、点自体が自動生成された場合でも、ユーザーはここで明示的なテクスチャ座標を指定できます。これには、自動生成された点のレイアウトと、それらがマテリアルによって参照されるテクスチャにどのように対応するかを理解する必要があります。

.. _body-flexcomp-mass:

:at:`mass`: :at-val:`real(1), "1"`
   自動生成される各ボディの質量は、この値を点の数で割ったものに等しくなります。一部の点を固定しても、他のボディの質量には影響しないことに注意してください。

.. _body-flexcomp-inertiabox:

:at:`inertiabox`: :at-val:`real(1), "0.005"`
   自動生成されるボディは、スライダージョイントを持つ点質量の物理を持っていますが、MuJoCo では各ボディが回転慣性を持つ必要があります。ここで生成される慣性は対角であり、対応する等価慣性ボックスの辺がこの値に等しくなるように計算されます。

.. _body-flexcomp-file:

:at:`file`: :at-val:`string, optional`
   **表面** （三角形）または **体積** （四面体）メッシュが読み込まれるファイルの名前です。表面メッシュの場合、ファイル拡張子を使用してファイル形式が決定されます。サポートされる形式は、GMSH と :ref:`mesh assets<asset-mesh>` で指定される形式（レガシーの .msh 形式を除く）です。体積メッシュは GMSH 形式でのみサポートされます。GMSH ファイルの詳細については、 :ref:`こちら<gmsh-file-docs>` を参照してください。

.. _body-flexcomp-rigid:

:at:`rigid`: :at-val:`[true, false], "false"`
   これが true の場合、すべての点が親ボディ内の頂点に対応し、新しいボディは作成されません。これは、すべての点を固定することと同等です。実際にすべての点が固定されている場合、モデルコンパイラはフレックスが剛体であることを検出します（これは衝突検出で非凸メッシュとして動作します）。

.. _body-flexcomp-pos:

:at:`pos`: :at-val:`real(3), "0 0 0"`
   この 3D ベクトルは、すべての点を親ボディのフレームに対して平行移動します。

.. _body-flexcomp-quat:

:at:`quat`: :at-val:`real(4), "1 0 0 0"`
   これは、上で指定された :at:`pos` ベクトルの周りのすべての点のクォータニオン回転です。これら 2 つのベクトルは一緒にポーズ変換を定義し、必要に応じて点を配置および方向付けるために使用されます。

.. _body-flexcomp-axisangle:
.. _body-flexcomp-xyaxes:
.. _body-flexcomp-zaxis:
.. _body-flexcomp-euler:

:at:`axisangle`, :at:`xyaxes`, :at:`zaxis`, :at:`euler`
   :at:`quat` の代わりに使用できる回転の代替指定です。

.. _body-flexcomp-scale:

:at:`scale`: :at-val:`real(3), "1 1 1"`
   座標を明示的に指定するタイプの、すべての点座標のスケーリングです。スケーリングは、ポーズ変換の後に適用されます。

.. _body-flexcomp-radius:
.. _body-flexcomp-material:
.. _body-flexcomp-rgba:
.. _body-flexcomp-group:
.. _body-flexcomp-flatskin:

:at:`radius`, :at:`material`, :at:`rgba`, :at:`group`, :at:`flatskin`
   これらの属性は、自動生成される :ref:`flex<deformable-flex>` オブジェクトに直接渡され、同じ意味を持ちます。

.. _body-flexcomp-origin:

:at:`origin`: :at-val:`real(3), "0 0 0"`
   flexcomp の原点です。OBJ 表面メッシュから体積メッシュを生成するために使用されます。各表面三角形は原点に接続されて四面体を作成するため、結果として得られる体積メッシュは凸形状に対してのみ整形式であることが保証されます。

.. _flexcomp-contact:

:el-prefix:`flexcomp/` |-| **contact** |m|
''''''''''''''''''''''''''''''''''''''''''

.. _flexcomp-contact-internal:
.. _flexcomp-contact-selfcollide:
.. _flexcomp-contact-vertcollide:
.. _flexcomp-contact-activelayers:
.. _flexcomp-contact-contype:
.. _flexcomp-contact-conaffinity:
.. _flexcomp-contact-condim:
.. _flexcomp-contact-priority:
.. _flexcomp-contact-friction:
.. _flexcomp-contact-solmix:
.. _flexcomp-contact-solref:
.. _flexcomp-contact-solimp:
.. _flexcomp-contact-margin:
.. _flexcomp-contact-gap:
.. _flexcomp-contact-passive:

.. |body/flexcomp/contact attrib list| replace::
   :at:`internal`, :at:`selfcollide`, :at:`vertcollide`, :at:`activelayers`, :at:`contype`, :at:`conaffinity`,
   :at:`condim`, :at:`priority`, :at:`friction`, :at:`solmix`, :at:`solimp`, :at:`margin`, :at:`gap`

|body/flexcomp/contact attrib list|
   :ref:`flex/contact<flex-contact>` と同じです。すべての属性は、自動生成されるフレックスに渡されます。

.. _flexcomp-edge:

:el-prefix:`flexcomp/` |-| **edge** |m|
'''''''''''''''''''''''''''''''''''''''

各フレックス要素は、1D では 1 つのエッジ（カプセル要素と一致）、2D では 3 つのエッジ、3D では 6 つのエッジを持ちます。エッジは、フレックス要素がコンパイルされるときに自動的に生成され、ユーザーが直接指定することはできません。この要素は、フレックス内のすべてのエッジのプロパティを調整するために使用されます。

.. _flexcomp-edge-equality:

:at:`equality`: :at-val:`[false, true, vert], "false"`
   このエッジに適用される等式制約のタイプです。 **false** の場合、等式制約は適用されません。 **true** の場合、エッジ制約が強制されます。 **vert** の場合、平均化された制約が使用されます。 :ref:`flexvert<equality-flexvert>` を参照してください。

.. _flexcomp-edge-solref:
.. _flexcomp-edge-solimp:

:at:`solref`, :at:`solimp`
   標準制約パラメータで、自動生成される等式制約に渡されます。

.. _flexcomp-edge-stiffness:
.. _flexcomp-edge-damping:

:at:`stiffness`, :at:`damping`
   エッジの剛性と減衰で、自動生成されるフレックスに渡されます。


.. _flexcomp-elasticity:

:el-prefix:`flexcomp/` |-| **elasticity** |m|
'''''''''''''''''''''''''''''''''''''''''''''

.. _flexcomp-elasticity-young:
.. _flexcomp-elasticity-poisson:
.. _flexcomp-elasticity-damping:
.. _flexcomp-elasticity-thickness:
.. _flexcomp-elasticity-elastic2d:

.. |body/flexcomp/elasticity attrib list| replace::
   :at:`young`, :at:`poisson`, :at:`damping`, :at:`thickness`, :at:`elastic2d`

|body/flexcomp/elasticity attrib list|
   :ref:`flex/elasticity<flex-elasticity>` と同じです。
   すべての属性は、自動生成されるフレックスに渡されます。

.. _flexcomp-pin:

:el-prefix:`flexcomp/` |-| **pin** |m|
''''''''''''''''''''''''''''''''''''''

各点は固定されているか、固定されていないかのいずれかです。固定の効果については前述しました。この要素は、どの点が固定されているかを指定するために使用されます。以下の各属性は複数のピンを指定するために使用でき、さらに、 :el:`pin` 要素自体をユーザーの利便性のために繰り返すことができます。効果は累積的です。同じ点を複数回固定することは許可されます。

.. _flexcomp-pin-id:

:at:`id`: :at-val:`int(n), required`
   固定する点の 0 ベースの ID です。点が自動生成される場合、ユーザーはどの点を固定するかを決定するために、それらのレイアウトを理解する必要があります。これは、最初にピンなしで flexcomp を作成し、シミュレータでロードし、ボディラベルを表示することで実行できます。

.. _flexcomp-pin-range:

:at:`range`: :at-val:`int(2*n), required`
   固定する点の範囲です。各範囲は 2 つの整数で指定されます。

.. _flexcomp-pin-grid:

:at:`grid`: :at-val:`int(dim*n), required`
   固定する点のグリッド座標です。これは type grid でのみ使用できます。

.. _flexcomp-pin-gridrange:

:at:`gridrange`: :at-val:`int(2*dim*n), required`
   固定する点のグリッド座標の範囲です。各範囲は、範囲の最小値を指定する (dim) 個の整数と、それに続く範囲の最大値を指定する (dim) 個の整数で指定されます。これは type grid でのみ使用できます。

.. _flexcomp-plugin:

:el-prefix:`flexcomp/` |-| **plugin** |?|
'''''''''''''''''''''''''''''''''''''''''

この flexcomp を :ref:`engine plugin<exPlugin>` に関連付けます。 :at:`plugin` または :at:`instance` のいずれかが必要です。

.. _flexcomp-plugin-plugin:

:at:`plugin`: :at-val:`string, optional`
   プラグイン識別子。暗黙的なプラグインインスタンス化に使用されます。

.. _flexcomp-plugin-instance:

:at:`instance`: :at-val:`string, optional`
   インスタンス名。明示的なプラグインインスタンス化に使用されます。


.. _body-plugin:

:el-prefix:`body/` |-| **plugin** |?|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

このボディを :ref:`engine plugin<exPlugin>` に関連付けます。 :at:`plugin` または :at:`instance` のいずれかが必要です。

.. _body-plugin-plugin:

:at:`plugin`: :at-val:`string, optional`
   プラグイン識別子。暗黙的なプラグインインスタンス化に使用されます。

.. _body-plugin-instance:

:at:`instance`: :at-val:`string, optional`
   インスタンス名。明示的なプラグインインスタンス化に使用されます。


.. _body-attach:

:el-prefix:`body/` |-| **attach** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

:el:`attach` 要素は、別のモデルからボディのサブツリーを、このモデルのキネマティックツリーに挿入するために使用されます。パーサーで実装され、あるファイルから別のファイルへ XML をコピーアンドペーストすることと同等である :ref:`include<include>` とは異なり、 :el:`attach` はモデルコンパイラで実装されます。この要素を使用するには、サブモデルを最初に :ref:`asset<asset-model>` として定義する必要があります。アタッチメントを作成する際、アタッチされたサブツリーのトップボディが指定され、キネマティックツリー外のすべての参照要素（センサーやアクチュエータなど）も、トップレベルモデルにコピーされます。さらに、アタッチされたサブツリー内から参照される要素（デフォルトやアセットなど）も、トップレベルモデルにコピーされます。 :el:`attach` は :ref:`meta-element` であるため、保存時にはすべてのアタッチメントが保存された XML ファイルに表示されます。この要素は、プロシージャルな :ref:`attachment<meAttachment>` 機能のサブセットであることに注意してください。そのため、そこで説明されているのと同じ制限を共有します。さらに、 :el:`attach` 要素が使用される場合、モデル全体（つまり、参照されているか否かに関わらず、すべての要素を含む）をアタッチすることはできません。

.. _body-attach-model:

:at:`model`: :at-val:`string, required`
   サブツリーをアタッチする元のサブモデルです。

.. _body-attach-body:

:at:`body`: :at-val:`string, optional`
   ここにアタッチするサブモデル内のボディの名前です。ボディとそのサブツリーがアタッチされます。この属性が指定されていない場合、world ボディの内容が新しい :ref:`frame<body-frame>` にアタッチされます。

.. _body-attach-prefix:

:at:`prefix`: :at-val:`string, required`
   サブモデル内の要素の名前の前に付加するプレフィックスです。この属性は、親との名前の競合や、同じサブツリーを複数回アタッチする場合の名前の競合を防ぐために必要です。


.. _body-frame:

:el-prefix:`body/` |-| **frame** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

フレームは、すべての子要素に適用される座標変換を指定します。フレームはコンパイル時に消失し、エンコードされた変換は直接の子に累積されます。例については :ref:`frame<frame>` を参照してください。

.. _frame-name:

:at:`name`: :at-val:`string, optional`
   フレームの名前です。

.. _frame-childclass:

:at:`childclass`: :at-val:`string, optional`
   この属性が存在する場合、デフォルトクラスを許容するすべての子孫要素は、ここで指定されたクラスを使用します。ただし、それら自身のクラスを指定するか、childclass 属性を持つ別のフレームまたはボディが、ネストされたボディとフレームのチェーンに沿って遭遇する場合を除きます。 :ref:`CDefault` を参照してください。

.. _frame-pos:

:at:`pos`: :at-val:`real(3), "0 0 0"`
   親座標系におけるフレームの 3D 位置です。

.. _frame-quat:

.. _frame-axisangle:

.. _frame-xyaxes:

.. _frame-zaxis:

.. _frame-euler:

:at:`quat`, :at:`axisangle`, :at:`xyaxes`, :at:`zaxis`, :at:`euler`
   :ref:`COrientation` を参照してください。



.. _contact:

**contact** |m|
~~~~~~~~~~~~~~~

これはグループ化要素であり、属性を持ちません。衝突チェックのための候補接触ペアの生成を調整するために使用される要素をグループ化します。 :ref:`Collision` については、Computation 章で詳細に説明されているため、ここでの説明は簡潔です。


.. _contact-pair:

:el-prefix:`contact/` |-| **pair** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

この要素は、衝突がチェックされる事前定義されたジオムペアを作成します。対応するジオムプロパティからプロパティが推測される動的に生成されるペアとは異なり、ここで作成されるペアは、すべてのプロパティを明示的に、またはデフォルトを通じて指定し、個々のジオムのプロパティは使用されません。異方性摩擦は、この要素でのみ作成できます。

.. _contact-pair-name:

:at:`name`: :at-val:`string, optional`
   この接触ペアの名前です。

.. _contact-pair-class:

:at:`class`: :at-val:`string, optional`
   未指定の属性を設定するためのデフォルトクラスです。

.. _contact-pair-geom1:

:at:`geom1`: :at-val:`string, required`
   ペアの最初のジオムの名前です。

.. _contact-pair-geom2:

:at:`geom2`: :at-val:`string, required`
   ペアの 2 番目のジオムの名前です。ソルバーによって計算され、mjData.efc_force に格納される接触力ベクトルは、慣例により、最初のジオムから 2 番目のジオムに向かって指します。システムに適用される力はもちろん大きさが等しく逆向きであるため、ジオムの順序は物理に影響を与えません。

.. _contact-pair-condim:

:at:`condim`: :at-val:`int, "3"`
   このジオムペアによって生成される接触の次元です。

.. _contact-pair-friction:

:at:`friction`: :at-val:`real(5), "1 1 0.005 0.0001 0.0001"`
   このジオムペアによって生成される接触の摩擦係数です。最初の 2 つの係数を異なる値にすると、異方性接線摩擦が生じます。最後の 2 つの係数を異なる値にすると、異方性転がり摩擦が生じます。この配列の長さはパーサーによって強制されず、5 より小さくすることができます。これは、接触の次元に応じて、一部の係数が使用されない可能性があるためです。未指定の係数は、デフォルトと等しいままです。

.. _contact-pair-solref:

.. _contact-pair-solimp:

:at:`solref`, :at:`solimp`
   接触シミュレーションの制約ソルバーパラメータです。 :ref:`CSolver` を参照してください。

.. _contact-pair-solreffriction:

:at:`solreffriction`: :at-val:`real, "0 0"`
   摩擦次元における接触参照加速度です。この属性は、他の :at:`solref` 属性（ :ref:`CSolver` で説明）と同じセマンティクスを持ちますが、2 つの重要な違いがあります：

   - デフォルトの :at-val:`"0 0"` は「 :at:`solref` と同じ値を使用する」ことを意味します。
   - この属性は、 :ref:`elliptic friction cones<option-cone>` に対してのみ有効です。角錐摩擦円錐は、法線方向と摩擦方向の力を混合するためです。

   他の :at:`solreffriction` 属性と同様に、制約違反は同一に 0 であることに注意してください。したがって、正のセマンティクスを使用する場合、 :at:`solreffriction[1]` は無視され、負のセマンティクスの場合、 :at:`solreffriction[0]` が無視されます。詳細については、 :ref:`CSolver` を参照してください。

.. _contact-pair-margin:

:at:`margin`: :at-val:`real, "0"`
   接触が検出され、グローバル配列 mjData.contact に含まれる距離閾値です。

.. _contact-pair-gap:

:at:`gap`: :at-val:`real, "0"`
   この属性は、非アクティブな接触、つまり制約ソルバーによって無視されるが、カスタム計算の目的で mjData.contact に含まれる接触の生成を有効にするために使用されます。この値が正の場合、margin と margin-gap の間のジオム距離は、そのような非アクティブな接触に対応します。


.. _contact-exclude:

:el-prefix:`contact/` |-| **exclude** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

この要素は、衝突チェックからボディのペアを除外するために使用されます。ジオムを参照する他のすべての接触関連要素とは異なり、この要素はボディを参照します。経験上、除外はボディのレベルでより有用であることが示されています。最初のボディで定義されたジオムと、2 番目のボディで定義されたジオムとの間の衝突は除外されます。

.. _contact-exclude-name:

:at:`name`: :at-val:`string, optional`
   この除外ペアの名前です。

.. _contact-exclude-body1:

:at:`body1`: :at-val:`string, required`
   ペアの最初のボディの名前です。

.. _contact-exclude-body2:

:at:`body2`: :at-val:`string, required`
   ペアの 2 番目のボディの名前です。


.. _deformable:

**deformable** |m|
~~~~~~~~~~~~~~~~~~

これはグループ化要素であり、属性はありません。変形可能オブジェクト、すなわちフレックスとスキンを指定する要素をグループ化します。


.. _deformable-flex:

:el-prefix:`deformable/` |-| **flex** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

フレキシブルオブジェクト（またはフレックス）は MuJoCo 3.0 で追加されました。これらは、異なる移動するボディフレーム内で定義された頂点を接続する、質量のない伸縮可能な幾何要素（カプセル、三角形、または四面体）の集合です。これらの伸縮可能な要素は衝突と接触力をサポートし、その力はすべての相互接続されたボディに分配されます。フレックスはまた、必要に応じて受動力と制約力を生成し、望ましい材料特性を持つ変形可能エンティティをシミュレートします。フレックスのモデリングは、 :ref:`flexcomp<body-flexcomp>` 要素によって自動化され、簡略化されています。ほとんどの場合、ユーザーは :el:`flexcomp` を指定し、それが対応する低レベルの :el:`flex` を自動的に構築します。詳細については、 :ref:`変形可能オブジェクト<CDeformable>` を参照してください。

.. _deformable-flex-name:

:at:`name`: :at-val:`string, optional`
   フレックスの名前。

.. _deformable-flex-dim:

:at:`dim`: :at-val:`int, "2"`
   フレックスの次元。許可される値は 1、2、3 です。1D では要素はカプセル、2D では要素は半径付き三角形、3D では要素は（オプションで）半径付き四面体です。

.. _deformable-flex-radius:

:at:`radius`: :at-val:`real, "0.005"`
   すべてのフレックス要素の半径。3D ではゼロにできますが、1D と 2D では正でなければなりません。半径は衝突検出とレンダリングの両方に影響します。1D と 2D では、要素を体積化するために必要です。

.. _deformable-flex-body:

:at:`body`: :at-val:`string(nvert or 1), required`
   各頂点が属する MuJoCo ボディ名の配列（空白で区切られる）。ボディ名の数は、頂点数（nvert）と等しいか、または単一のボディでなければなりません。単一のボディが指定された場合、すべての頂点はそのボディ内で定義されます。この場合、フレックスは剛体になります。後者の機能は、事実上一般的な非凸メッシュを作成します（メッシュジオムは衝突検出目的で凸化されるのとは異なります）。

.. _deformable-flex-vertex:

:at:`vertex`: :at-val:`real(3*nvert), optional`
   対応するボディフレーム内の頂点のローカル座標。この属性が省略された場合、すべての座標は (0,0,0) です。つまり、頂点はボディフレームの中心と一致します。

.. _deformable-flex-texcoord:

:at:`texcoord`: :at-val:`real(2*vert or ntexcoord), optional`
   テクスチャ座標。省略された場合、マテリアルでテクスチャが指定されていても、このフレックスのテクスチャマッピングは無効になります。

.. _deformable-flex-elemtexcoord:

:at:`elemtexcoord`: :at-val:`int((dim+1)*nelem), optional`
   各面のテクスチャインデックス。省略された場合、テクスチャは頂点ベースであると仮定されます。

.. _deformable-flex-element:

:at:`element`: :at-val:`int((dim+1)*nelem), required`
   フレックスの各要素について、そのフレックス要素を形成する頂点のゼロベースのインデックスをリストします。カプセルを指定するには 2 つの頂点、三角形を指定するには 3 つの頂点、四面体を指定するには 4 つの頂点が必要です。これが、インデックスの数が (dim+1) に要素数を掛けたものと等しい理由です。2D では、頂点は反時計回りの順序でリストする必要があります。1D と 3D では順序は無関係です。3D では、モデルコンパイラが必要に応じて頂点を並べ替えます。フレックス要素内での頂点インデックスの重複は許可されていません。フレックスのトポロジーは強制されません。連続した柔軟体、または切り離された伸縮可能な要素の集合、またはその中間に対応する可能性があります。

.. _deformable-flex-flatskin:

:at:`flatskin`: :at-val:`[true, false], "false"`
   この属性は、flexskin モードでレンダリングされる 2D および 3D フレックスが、スムーズシェーディングまたはフラットシェーディングを使用するかどうかを決定します。デフォルトのスムーズシェーディングはほとんどの場合に適していますが、オブジェクトが鋭いエッジを持つことを意図している場合（立方体など）、フラットシェーディングがより自然です。

.. _deformable-flex-material:

:at:`material`: :at-val:`string, optional`
   指定された場合、この属性はフレックスに :ref:`マテリアル<asset-material>` を適用します。マテリアルで指定されたテクスチャは、フレックスが明示的なテクスチャ座標を持つ場合にのみ適用されることに注意してください。

.. _deformable-flex-rgba:

:at:`rgba`: :at-val:`real(4), "0.5 0.5 0.5 1"`
   マテリアルアセットを作成して参照する代わりに、この属性を使用して色と透明度のみを設定できます。これはマテリアルメカニズムほど柔軟ではありませんが、より便利で、多くの場合十分です。この属性の値が内部デフォルトと異なる場合、マテリアルよりも優先されます。

.. _deformable-flex-group:

:at:`group`: :at-val:`int, "0"`
   フレックスが属する整数グループ。この属性はカスタムタグに使用できます。また、ビジュアライザーによって、フレックスのグループ全体のレンダリングを有効/無効にするために使用されます。

.. _deformable-flex-node:

:at:`node`: :at-val:`string(nnode), optional`
   フレックスの自由度。各ノードが属する MuJoCo ボディ名の配列（空白で区切られる）。ボディ名の数はノード数（nnode）と等しくなければなりません。詳細については、flexcomp の :ref:`dof<body-flexcomp-dof>` 属性を参照してください。

.. _flex-edge:

:el-prefix:`flex/` |-| **edge** |?|
'''''''''''''''''''''''''''''''''''

この要素は、フレックスのすべてのエッジの受動特性または制約特性を調整します。フレックスエッジには、減衰受動力とそれに関連する :ref:`等式制約<equality-flex>` を持つことができ、エッジ制約力が生成されます。後者の場合、受動力は通常不要です。1D フレックスの場合、エッジは受動剛性を持つこともできますが、2D および 3D の場合は ``Solid`` または ``Membrane`` ファーストパーティプラグインを使用できます。これにより、通常はエッジ制約が不要になります。ただし、これらはユーザーに任されたモデリングの選択です。MuJoCo では、これらのメカニズムを必要に応じて組み合わせることができます。

.. _flex-edge-stiffness:

:at:`stiffness`: :at-val:`real(1), "0"`
   すべてのエッジの剛性。1D フレックスのみ。2D および 3D の場合は、プラグインを使用する必要があります。

.. _flex-edge-damping:

:at:`damping`: :at-val:`real(1), "0"`
   すべてのエッジの減衰。


.. _flex-elasticity:

:el-prefix:`flex/` |-| **elasticity** |?|
'''''''''''''''''''''''''''''''''''''''''

弾性モデルは、区分線形有限要素で離散化された `Saint Venant-Kirchhoff
<https://en.wikipedia.org/wiki/Hyperelastic_material#Saint_Venant%E2%80%93Kirchhoff_model>`__ モデルであり、大変位（有限回転）と小ひずみを受ける超弾性材料の圧縮または伸長をシミュレートすることを目的としています。非線形ひずみ・変位関係を使用しますが、線形応力・ひずみ関係を使用します。 :ref:`変形可能<CDeformable>` オブジェクトも参照してください。

.. _flex-elasticity-young:

:at:`young`: :at-val:`real(1), "0"`
   ヤング弾性率。連続弾性材料の引張剛性と圧縮剛性の尺度です。単位は :math:`\textrm{pressure}=\textrm{force}/\textrm{area}` です。

.. _flex-elasticity-poisson:

:at:`poisson`: :at-val:`real(1), "0"`
   ポアソン比。適用された縦ひずみに対する横変形の比率です。この無次元量は :math:`[0, 0.5)` の範囲にあります。小さい値または大きい値は、それぞれ圧縮性または非圧縮性を意味します。

.. _flex-elasticity-damping:

:at:`damping`: :at-val:`real(1), "0"`
   レイリー減衰係数。単位は時間です。この量は、ヤング率によって定義された剛性をスケーリングして、減衰行列を生成します。

.. _flex-elasticity-thickness:

:at:`thickness`: :at-val:`real(1), "-1"`
   シェル厚。単位は長さです。2D フレックスのみで使用されます。引張剛性をスケーリングするために使用されます。この厚さは、ジオメトリに合わせるために :ref:`radius <deformable-flex-radius>` の 2 倍に設定できますが、半径は衝突検出に関連する考慮事項によって制約される可能性があるため、別に公開されています。

.. _flex-elasticity-elastic2d:

:at:`elastic2d`: :at-val:`[none, bend, stretch, both], "none"`
   2D フレックスの受動力への弾性寄与。"none": なし、"bend": 曲げのみ、"stretch": 引張のみ、"both": 曲げと引張の両方。

.. _flex-contact:

:el-prefix:`flex/` |-| **contact** |?|
''''''''''''''''''''''''''''''''''''''

この要素は、フレックスの接触特性を調整します。これはジオムの接触特性とほぼ同一ですが、フレックスに固有のいくつかの拡張があります。

.. _flex-contact-internal:

:at:`internal`: :at-val:`[true, false], "false"`
   フレックスの自己貫通と要素反転を防ぐ内部衝突を有効または無効にします。共有頂点を持つフレックス要素は衝突できないことに注意してください（さもないと永続的な接触が発生します）。1D および 2D では、内部衝突チェックは事前定義された頂点・要素ペアに依存します。ここで、頂点はフレックスと同じ半径の球として扱われます。これらの球は、フレックスの周辺にある隣接要素の非共有頂点に対応します。事前定義された頂点・要素ペアは、モデルコンパイラによって自動的に生成されます。3D では、内部衝突チェックは各四面体内で実行されます。各頂点は、対向する三角形面に対応する平面と衝突します（再びフレックス半径を使用）。結果として生じる接触は、常に condim 1、gap 0、margin 0 で作成されます。内部接触は :ref:`弾性パラメータ<flex-elasticity>` によって暗示される動作を変更することに注意してください。要素反転を防ぐことができないフレックスにのみ推奨されます。この属性のデフォルト値は、バージョン 3.3.1 で "true" から "false" に変更されました。

.. _flex-contact-selfcollide:

:at:`selfcollide`: :at-val:`[none, narrow, bvh, sap, auto], "auto"`
   これは、同じフレックスに属する要素ペアのミッドフェーズ衝突プルーニングの戦略を決定します。 **none** は、フレックス要素が相互に衝突できないことを意味します。 **narrow** は、ナローフェーズのみを意味します（つまり、すべてのペアがチェックされます）。これは診断ツールであり、実際には決して良いアイデアではありません。 **bvh** と **sap** は、バウンディングボリューム階層とスイープ・アンド・プルーン（ミッドフェーズ衝突プルーニングの 2 つの異なる戦略）を指します。 **auto** は、1D および 2D では **sap** を選択し、3D では **bvh** を選択します。どちらの戦略がより優れたパフォーマンスを発揮するかは、モデルの詳細に依存します。自動設定は、一般的に良好なパフォーマンスを発揮することがわかっている単純なルールです。

.. _flex-contact-vertcollide:

:at:`vertcollide`: :at-val:`[true, false], "false"`
   頂点衝突を有効または無効にします。 **true** の場合、フレックスの頂点に、フレックスの半径と等しい半径の球状ジオムが追加されます。これらのジオムは他のジオムと衝突でき、デフォルトでは表示されません。 **false** の場合、追加のジオムは追加されません。

.. _flex-contact-activelayers:

:at:`activelayers`: :at-val:`int(1), "1"`
   これは 3D フレックスのみに影響します。各四面体は、モデルコンパイラによって、フレックスの外側表面までの（グラフ）距離に対応する整数でラベル付けされます。したがって、外側を向いている要素はレイヤー 0 にあり、それらの隣接要素はレイヤー 1 にあります。この属性は、衝突に参加できるレイヤーの数を指定します。デフォルト設定 1 は、1 つのレイヤー（つまり、レイヤー 0）のみが、それ自体および世界の残りの部分と衝突できることを意味します。これは通常十分ですが、外側のレイヤーが小さな四面体で構成されている場合、別のボディがそれを「突き刺し」、スタックする可能性があります。その場合、値を増やす必要があります。


.. _flex-contact-contype:
.. _flex-contact-conaffinity:
.. _flex-contact-condim:
.. _flex-contact-priority:
.. _flex-contact-friction:
.. _flex-contact-solmix:
.. _flex-contact-solref:
.. _flex-contact-solimp:
.. _flex-contact-margin:
.. _flex-contact-gap:

.. |deformable/flex/contact attrib list| replace::
   :at:`contype`, :at:`conaffinity`, :at:`condim`, :at:`priority`, :at:`friction`,
   :at:`solmix`, :at:`solref`, :at:`solimp`, :at:`margin`, :at:`gap`

|deformable/flex/contact attrib list|
   通常の :ref:`ジオム <body-geom>` 属性と同じ意味です。

.. _flex-contact-passive:

:at:`passive`: :at-val:`[true, false], "false"`
   有効にすると、接触は接触ソルバーに追加されませんが、代わりに受動（バネ・ダンパー）接触力を計算するために使用されます。すべての接触は、指定された condim に関係なく、摩擦のない（condim 1）ものです。これは実験的な機能であり、将来のリリースで変更される可能性があります。


.. _deformable-skin:

:el-prefix:`deformable/` |-| **skin** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

これらは、モデルがレンダリングされるたびに頂点位置と法線が計算される変形可能メッシュです。MuJoCo スキンは可視化にのみ使用され、物理演算には影響しません。特に、衝突にはスキンが接続されているボディのジオムが関与し、スキン自体は関与しません。ジオムから参照され、衝突に参加する通常のメッシュとは異なり、スキンはモデル内の他の場所から参照されません。これは、レンダラーによって使用され、シミュレーターによっては使用されないスタンドアロン要素です。

スキンは、実行時に更新される頂点位置と法線、および事前定義された三角形面とオプションのテクスチャ座標を持ちます。また、更新に使用される「ボーン」も持っています。ボーンは、 :el:`bone` サブ要素で参照される通常の MuJoCo ボディです。各ボーンには、頂点インデックスのリストと対応する実数値の重みがあり、ボーンの位置と向きが対応する頂点にどの程度影響するかを指定します。頂点は、それに影響を与えるすべてのボーンに対するローカル座標を持ちます。ローカル座標は、グローバル頂点座標と各ボディのグローバルバインドポーズが与えられた場合に、モデルコンパイラによって計算されます。バインドポーズは、モデル参照構成 qpos0 に対応する必要はありません。スキン定義で提供される頂点位置とボーンバインドポーズは、モデル自体がローカル座標で定義されている場合でも、常にグローバルであることに注意してください。

実行時に、それに影響を与える各ボーンに対する各頂点のローカル座標がグローバル座標に変換され、対応する重みに比例して平均化され、各頂点の単一の 3D 座標セットが取得されます。次に、結果として得られたグローバル頂点位置と面情報が与えられた場合、法線が自動的に計算されます。最後に、各頂点位置にその（計算された）法線に沿ったオフセットを適用することにより、スキンを膨張させることができます。スキンはレンダリング目的で片面です。これは、シェーディングとエイリアシングのアーティファクトを回避するために、バックフェースカリングが必要なためです。スキンが閉じた 3D 形状の場合、背面は見えないため、これは問題ではありません。ただし、スキンが 2D オブジェクトの場合、両側を指定し、アーティファクトを回避するためにわずかにオフセットする必要があります。複合オブジェクトは自動的にスキンを生成することに注意してください。したがって、複合オブジェクトを含む XML モデルを保存すると、スキンが XML でどのように指定されるかの詳細な例を取得できます。

メッシュと同様に、スキンは、後で文書化される属性を介して XML で直接指定することも、カスタム形式のバイナリ SKN ファイルからロードすることもできます。ボーンサブ要素のため、スキンの仕様はメッシュよりも複雑です。ファイル形式は、4 つの整数のヘッダーで始まります: nvertex、ntexcoord、nface、nbone です。最初の 3 つはメッシュと同じであり、スキン内の頂点の総数、テクスチャ座標ペア、および三角形面を指定します。ntexcoord はゼロまたは nvertex と等しくなります。nbone は、スキンでボーンとして使用される MuJoCo ボディの数を指定します。ヘッダーの後には、頂点、テクスチャ座標、および面データが続き、その後に各ボーンの仕様が続きます。ボーン仕様には、対応するモデルボディの名前、3D バインド位置、4D バインドクォータニオン、ボーンによって影響を受ける頂点の数、および頂点インデックス配列と重み配列が含まれます。ボディ名は固定長文字配列として表され、ゼロで終端されることが期待されます。最初のゼロの後の文字は無視されます。SKN ファイルの内容は次のとおりです:

.. code:: Text

       (int32)   nvertex
       (int32)   ntexcoord
       (int32)   nface
       (int32)   nbone
       (float)   vertex_positions[3*nvertex]
       (float)   vertex_texcoords[2*ntexcoord]
       (int32)   face_vertex_indices[3*nface]
       for each bone:
           (char)    body_name[40]
           (float)   bind_position[3]
           (float)   bind_quaternion[4]
           (int32)   vertex_count
           (int32)   vertex_index[vertex_count]
           (float)   vertex_weight[vertex_count]

MuJoCo で使用される他のカスタムバイナリ形式と同様に、ファイルサイズ（バイト単位）はモデルコンパイラによって厳密に強制されます。スキンファイル形式にはサブ要素があるため、全体的なファイルサイズの式を記述することは困難ですが、上記の仕様から明確であるはずです。

.. _deformable-skin-name:

:at:`name`: :at-val:`string, optional`
   スキンの名前。

.. _deformable-skin-file:

:at:`file`: :at-val:`string, optional`
   スキンがロードされる SKN ファイル。パスは、 :ref:`コンパイラ<compiler>` の meshdir 属性で説明されているように決定されます。ファイルが省略された場合、スキン仕様は以下の属性を使用して XML で提供する必要があります。

.. _deformable-skin-vertex:

:at:`vertex`: :at-val:`real(3*nvert), optional`
   スキンが定義されるグローバルバインドポーズでの頂点 3D 位置。

.. _deformable-skin-texcoord:

:at:`texcoord`: :at-val:`real(2*nvert), optional`
   頂点 2D テクスチャ座標。0 と 1 の間です。スキンとジオムのテクスチャリングはやや異なることに注意してください。ジオムは自動テクスチャ座標生成を使用できますが、スキンは使用できません。これは、スキンデータがグローバル座標で直接計算されるためです。したがって、マテリアルがテクスチャを参照する場合、この属性を使用してスキンに明示的なテクスチャ座標を指定する必要があります。そうしないと、スキンが移動している間、テクスチャが世界で静止しているように見えます（興味深い効果を生み出しますが、おそらく意図したものではありません）。

.. _deformable-skin-face:

:at:`face`: :at-val:`int(3*nface), optional`
   三角形スキン面。各面は、ゼロから nvert-1 の間の整数である頂点インデックスのトリプルです。

.. _deformable-skin-inflate:

:at:`inflate`: :at-val:`real, "0"`
   この数値がゼロでない場合、更新時の頂点の位置は頂点法線に沿ってオフセットされますが、この属性で指定された距離です。これは、柔軟な 2D 形状を表すスキンに特に役立ちます。

.. _deformable-skin-material:

:at:`material`: :at-val:`string, optional`
   指定された場合、この属性はスキンにマテリアルを適用します。

.. _deformable-skin-rgba:

:at:`rgba`: :at-val:`real(4), "0.5 0.5 0.5 1"`
   マテリアルアセットを作成して参照する代わりに、この属性を使用して色と透明度のみを設定できます。これはマテリアルメカニズムほど柔軟ではありませんが、より便利で、多くの場合十分です。この属性の値が内部デフォルトと異なる場合、マテリアルよりも優先されます。

.. _deformable-skin-group:

:at:`group`: :at-val:`int, "0"`
   スキンが属する整数グループ。この属性はカスタムタグに使用できます。また、ビジュアライザーによって、スキンのグループ全体のレンダリングを有効/無効にするために使用されます。


.. _skin-bone:

:el-prefix:`skin/` |-| **bone** |m|
'''''''''''''''''''''''''''''''''''

この要素は、スキンのボーンを定義します。ボーンは、ここで名前によって参照される通常の MuJoCo ボディです。


.. _skin-bone-body:

:at:`body`: :at-val:`string, required`
   このボーンに対応するボディの名前。

.. _skin-bone-bindpos:

:at:`bindpos`: :at-val:`real(3), required`
   バインドポーズに対応するグローバルボディ位置。

.. _skin-bone-bindquat:

:at:`bindquat`: :at-val:`real(4), required`
   バインドポーズに対応するグローバルボディ向き。

.. _skin-bone-vertid:

:at:`vertid`: :at-val:`int(nvert), required`
   このボーンによって影響を受ける頂点の整数インデックス。頂点インデックスは、スキンメッシュ内の頂点の順序に対応します。ここで指定された頂点インデックスの数（nvert）は、次の属性で指定された頂点重みの数と等しくなければなりません。同じ頂点が複数のボーンによって影響を受ける可能性があり、各頂点は少なくとも 1 つのボーンによって影響を受ける必要があります。

.. _skin-bone-vertweight:

:at:`vertweight`: :at-val:`real(nvert), required`
   このボーンによって影響を受ける頂点の重み。頂点インデックスと同じ順序です。負の重みは許可されます（たとえば、3 次補間に必要です）が、特定の頂点に対するすべてのボーン重みの合計は正でなければなりません。



.. _equality:

**equality** |m|
~~~~~~~~~~~~~~~~

これは等式制約のグループ化要素です。属性はありません。等式制約の詳細な説明については、計算の章の :ref:`等式制約<coEquality>` セクションを参照してください。いくつかの属性はすべての等式制約タイプに共通であるため、 :ref:`connect <equality-connect>` 要素の下でのみ文書化します。


.. _equality-connect:

:el-prefix:`equality/` |-| **connect** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

この要素は、2 つのボディを点で接続する等式制約を作成します。この制約は、キネマティックツリー外のボールジョイントを効果的に定義します。接続制約は、次の 2 つの方法のいずれかで指定できます。

- :ref:`body1<equality-connect-body1>` と :ref:`anchor<equality-connect-anchor>` （両方とも必須）およびオプションで :ref:`body2<equality-connect-body2>` を使用。この仕様を使用する場合、制約はモデルが定義されている構成（ ``mjData.qpos0`` ）で満たされると仮定されます。
- :ref:`site1<equality-connect-site1>` と :ref:`site2<equality-connect-site2>` （両方とも必須）。この仕様を使用する場合、デフォルト構成での位置に関係なく、2 つのサイトが制約によって引き寄せられます。この仕様の例は、 `このモデル <https://github.com/google-deepmind/mujoco/blob/main/test/engine/testdata/equality_site.xml>`__ に示されています。

.. _equality-connect-name:

:at:`name`: :at-val:`string, optional`
   等式制約の名前。

.. _equality-connect-class:

:at:`class`: :at-val:`string, optional`
   指定されていない属性を設定するためのデフォルトクラス。

.. _equality-connect-active:

:at:`active`: :at-val:`[false, true], "true"`
   この属性が "true" に設定されている場合、制約はアクティブであり、制約ソルバーはそれを強制しようとします。フィールド :ref:`mjModel.eq_active0<mjModel>` はこの値に対応し、 :ref:`mjData.eq_active<mjData>` を初期化するために使用されます。これは実行時にユーザーが設定可能です。

.. _equality-connect-solref:

.. _equality-connect-solimp:

:at:`solref`, :at:`solimp`
   等式制約シミュレーションの制約ソルバーパラメータ。 :ref:`CSolver` を参照してください。

.. _equality-connect-body1:

:at:`body1`: :at-val:`string, optional`
   制約に参加する最初のボディの名前。この属性と :at:`anchor` を指定するか、または :at:`site1` と :at:`site2` を指定する必要があります。

.. _equality-connect-body2:

:at:`body2`: :at-val:`string, optional`
   制約に参加する 2 番目のボディの名前。この属性が省略された場合、2 番目のボディはワールドボディです。

.. _equality-connect-anchor:

:at:`anchor`: :at-val:`real(3), optional`
   2 つのボディが接続される 3D アンカー点の座標。 :at:`body1` のローカル座標フレームで指定されます。制約は、モデルが定義されている構成（ ``mjData.qpos0`` ）で満たされると仮定されます。これにより、コンパイラは :at:`body2` の関連アンカー点を計算できます。

.. _equality-connect-site1:

:at:`site1`: :at-val:`string, optional`
   制約に参加する最初のボディに属するサイトの名前。指定された場合、 :at:`site2` も指定する必要があります。 （ :at:`site1` 、 :at:`site2` ）仕様は、ボディベースの仕様よりも柔軟な代替手段であり、2 つの点で異なります。第一に、サイトはデフォルト構成で重なる必要はありません。重ならない場合、サイトはシミュレーションの開始時に「スナップ」します。第二に、実行時に ``mjModel.site_pos`` のサイト位置を変更すると、制約の位置が正しく変更されます（つまり、 ``mjModel.eq_data`` の内容は、このセマンティクスが使用される場合、効果がありません）。

.. _equality-connect-site2:

:at:`site2`: :at-val:`string, optional`
   制約に参加する 2 番目のボディに属するサイトの名前。指定された場合、 :at:`site1` も指定する必要があります。詳細については、 :ref:`site1<equality-connect-site1>` の説明を参照してください。

.. _equality-weld:

:el-prefix:`equality/` |-| **weld** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

この要素は、溶接等式制約を作成します。これは、2 つのボディを相互に接続し、それらの間のすべての相対自由度を削除します（もちろん、MuJoCo の他のすべての制約と同様に、ソフトに）。2 つのボディは、互いに近い必要はありません。制約ソルバーによって強制される相対ボディ位置と向きは、モデルが定義されたものです。2 つのボディを剛体的に溶接することもできます。これは、ジョイント要素なしで、子ボディとして一方のボディを他方のボディの子として定義することによって行います。溶接制約は、次の 2 つの方法のいずれかで指定できます:

- :ref:`body1<equality-weld-body1>` （およびオプションで :ref:`anchor<equality-weld-anchor>` 、 :ref:`relpose<equality-weld-relpose>` 、 :ref:`body2<equality-weld-body2>` ）を使用。この仕様を使用する場合、制約はモデルが定義されている構成で満たされると仮定されます。
- :ref:`site1<equality-weld-site1>` と :ref:`site2<equality-weld-site2>` （両方とも必須）。この仕様を使用する場合、デフォルト構成での位置に関係なく、2 つのサイトのフレームが制約によって整列されます。この仕様の例は、 `このモデル <https://github.com/google-deepmind/mujoco/blob/main/test/engine/testdata/equality_site.xml>`__ に示されています。

.. _equality-weld-name:

.. _equality-weld-class:

.. _equality-weld-active:

.. _equality-weld-solref:

.. _equality-weld-solimp:

:at:`name`, :at:`class`, :at:`active`, :at:`solref`, :at:`solimp`
    :ref:`connect <equality-connect>` 要素と同じです。

.. _equality-weld-body1:

:at:`body1`: :at-val:`string, optional`
   制約に参加する最初のボディの名前。この属性を指定するか、または :at:`site1` と :at:`site2` を指定する必要があります。

.. _equality-weld-body2:

:at:`body2`: :at-val:`string, optional`
   2 番目のボディの名前。この属性が省略された場合、2 番目のボディはワールドボディです。ボディをワールドに溶接し、実行時に mjData.eq_active の対応するコンポーネントを変更することで、ボディを一時的に固定できます。

.. _equality-weld-relpose:

:at:`relpose`: :at-val:`real(7), "0 1 0 0 0 0 0"`
   この属性は、body1 に対する body2 の相対ポーズ（3D 位置の後に 4D クォータニオン向きが続く）を指定します。クォータニオン部分（つまり、ベクトルの最後の 4 つのコンポーネント）がすべてゼロの場合、デフォルト設定と同様に、この属性は無視され、相対ポーズは qpos0 のモデル参照ポーズに対応するものになります。異常なデフォルトは、すべての等式制約タイプが数値パラメータに同じデフォルトを共有するためです。

.. _equality-weld-anchor:

:at:`anchor`: :at-val:`real(3), "0 0 0"`
   body2 に対する溶接点の座標。 :at:`relpose` が指定されていない場合、このパラメータの意味は接続制約の場合と同じですが、body2 に対して相対的です。 :at:`relpose` が指定されている場合、body1 はポーズを使用してアンカー点を計算します。

.. _equality-weld-site1:

:at:`site1`: :at-val:`string, optional`
   制約に参加する最初のボディに属するサイトの名前。指定された場合、 :at:`site2` も指定する必要があります。 （ :at:`site1` 、 :at:`site2` ）仕様は、ボディベースの仕様よりも柔軟な代替手段であり、2 つの点で異なります。第一に、サイトはデフォルト構成で重なる必要はありません。重ならない場合、サイトはシミュレーションの開始時に「スナップ」します。第二に、実行時に ``mjModel.site_pos`` と ``mjModel.site_quat`` のサイト位置と向きを変更すると、制約の位置と向きが正しく変更されます（つまり、 ``mjModel.eq_data`` の内容は、このセマンティクスが使用される場合、 :ref:`torquescale<equality-weld-torquescale>` を除いて効果がありません）。

.. _equality-weld-site2:

:at:`site2`: :at-val:`string, optional`
   制約に参加する 2 番目のボディに属するサイトの名前。指定された場合、 :at:`site1` も指定する必要があります。詳細については、 :ref:`site1<equality-weld-site1>` の説明を参照してください。

.. _equality-weld-torquescale:

:at:`torquescale`: :at-val:`real, "1"`
   角度残差（角度制約違反）をスケーリングする定数。概念的には :math:`\textrm{torque}/\textrm{force}=\textrm{length}` の単位です。直感的に、この係数は、溶接が並進変位に対する回転変位を「気にする」度合いを定義します。この値を 0 に設定すると、 :el:`weld` は :el:`connect` 制約のように動作します。この値には長さの単位があるため、次のように理解できることに注意してください。溶接が 2 つのボディを貼り付ける平らな接着剤のパッチによって実装されていると想像すると、 :at:`torquescale` はこの接着剤パッチの直径として解釈できます。


.. _equality-joint:

:el-prefix:`equality/` |-| **joint** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

この要素は、1 つのジョイントの位置または角度を別のジョイントの 4 次多項式に制約します。スカラージョイントタイプ（スライドとヒンジ）のみを使用できます。


.. _equality-joint-name:

.. _equality-joint-class:

.. _equality-joint-active:

.. _equality-joint-solref:

.. _equality-joint-solimp:

:at:`name`, :at:`class`, :at:`active`, :at:`solref`, :at:`solimp`
    :ref:`connect <equality-connect>` 要素と同じです。

.. _equality-joint-joint1:

:at:`joint1`: :at-val:`string, required`
   最初のジョイントの名前。

.. _equality-joint-joint2:

:at:`joint2`: :at-val:`string, optional`
   2 番目のジョイントの名前。この属性が省略された場合、最初のジョイントは定数に固定されます。

.. _equality-joint-polycoef:

:at:`polycoef`: :at-val:`real(5), "0 1 0 0 0"`
   4 次多項式の係数 :math:`a_0 \ldots a_4` 。 :at:`joint1` と :at:`joint2` のジョイント値がそれぞれ :math:`y` と :math:`x` であり、それらの参照位置（初期モデル構成のジョイント値に対応）が :math:`y_0` と :math:`x_0` である場合、制約は次のようになります:

   .. math::
      y-y_0 = a_0 + a_1(x-x_0) + a_2(x-x_0)^2 + a_3(x-x_0)^3 + a_4(x-x_0)^4

    :at:`joint2` を省略することは、 :math:`x = x_0` を設定することと同等であり、その場合、制約は :math:`y = y_0 + a_0` になります。

.. _equality-tendon:

:el-prefix:`equality/` |-| **tendon** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

この要素は、1 つのテンドンの長さを別のテンドンの 4 次多項式に制約します。


.. _equality-tendon-name:

.. _equality-tendon-class:

.. _equality-tendon-active:

.. _equality-tendon-solref:

.. _equality-tendon-solimp:

:at:`name`, :at:`class`, :at:`active`, :at:`solref`, :at:`solimp`
    :ref:`connect <equality-connect>` 要素と同じです。

.. _equality-tendon-tendon1:

:at:`tendon1`: :at-val:`string, required`
   最初のテンドンの名前。

.. _equality-tendon-tendon2:

:at:`tendon2`: :at-val:`string, optional`
   2 番目のテンドンの名前。この属性が省略された場合、最初のテンドンは定数に固定されます。

.. _equality-tendon-polycoef:

:at:`polycoef`: :at-val:`real(5), "0 1 0 0 0"`
   上記の :ref:`equality/joint <equality-joint>` 要素と同じですが、ジョイント位置の代わりにテンドン長に適用されます。


.. _equality-flex:

:el-prefix:`equality/` |-| **flex** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

この要素は、指定されたフレックスのすべてのエッジの長さを、初期モデル構成でのそれぞれの長さに制約します。この方法で、エッジは変形可能エンティティの形状を維持するために使用されます。他のすべての等式制約タイプは固定数のスカラー制約を追加しますが、この要素は指定されたフレックスのエッジ数と同じ数のスカラー制約を追加することに注意してください。

.. _equality-flex-name:
.. _equality-flex-class:
.. _equality-flex-active:
.. _equality-flex-solref:
.. _equality-flex-solimp:

:at:`name`, :at:`class`, :at:`active`, :at:`solref`, :at:`solimp`
    :ref:`connect <equality-connect>` 要素と同じです。

.. _equality-flex-flex:

:at:`flex`: :at-val:`string, required`
   エッジが制約されているフレックスの名前。


.. _equality-flexvert:

:el-prefix:`equality/` |-| **flexvert** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

この要素は、ひずみテンソルのトレースと行列式を、Chen, Kry, and Vouga, "Locking-free Simulation of Isometric Thin Plates", 2019 のように単位行列のものに制約します。ひずみテンソルは三角形ごとに計算され、頂点に隣接するすべての三角形で平均化されます。これにより、制約の数が 2T から 2V に減少し、ロッキングを回避するために V 自由度が解放されます。次元 2、つまり布状のフレックスのみでサポートされています。

.. _equality-flexvert-name:
.. _equality-flexvert-class:
.. _equality-flexvert-active:
.. _equality-flexvert-solref:
.. _equality-flexvert-solimp:

:at:`name`, :at:`class`, :at:`active`, :at:`solref`, :at:`solimp`
    :ref:`connect <equality-connect>` 要素と同じです。

.. _equality-flexvert-flex:

:at:`flex`: :at-val:`string, required`
   頂点が制約されているフレックスの名前。


.. _equality-distance:

:el-prefix:`equality/` |-| **distance** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

距離等式制約は MuJoCo バージョン 2.2.2 で削除されました。以前のバージョンを使用している場合は、対応するバージョンのドキュメントに切り替えてください。


.. _tendon:

**tendon** |m|
~~~~~~~~~~~~~~

テンドン定義のグループ化要素。固定テンドンの属性は空間テンドンの属性のサブセットであるため、空間テンドンの下で一度だけ文書化します。テンドンは、長さ制限を課し、バネ、減衰、乾燥摩擦力をシミュレートし、それらにアクチュエータを接続するために使用できます。等式制約で使用する場合、テンドンは異なる形式の機械的結合を表すこともできます。


.. _tendon-spatial:

:el-prefix:`tendon/` |-| **spatial** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. figure:: images/XMLreference/tendon.png
   :width: 400px
   :align: right

この要素は、指定した経由点を通り、指定した障害物ジオムの周りを巻き付く最小長さの経路を持つ空間テンドンを作成します。経路上のオブジェクトは、下記のサブ要素 :ref:`site <spatial-site>` と :ref:`geom <spatial-geom>` で定義します。また :ref:`pulley <spatial-pulley>` を定義することで、経路を複数の分岐に分割できます。テンドン経路の各分岐は、サイトで始まりサイトで終わる必要があり、複数の障害物ジオムを含む場合は、サイトで区切る必要があります。これは、テンドンレベルで反復ソルバーの必要性を回避するためです。この例は、アクチュエータの代わりにカウンターウェイトを持つ、指の伸筋として作用する多分岐テンドンを示しています: `tendon.xml <_static/tendon.xml>`__。

巻き付きの第2の形式は、テンドンが周囲を巻き付くのではなく、ジオムの *内部* を通過するよう制約される場合です。これは、サイドサイトが指定され、その位置が障害物ジオムの体積内部にある場合に自動的に有効化されます。

.. youtube:: I2q7D0Vda-A
   :width: 300px
   :align: right

**可視化:** テンドン経路は上図のように可視化され、以下の :ref:`width<tendon-spatial-width>`、 :ref:`material<tendon-spatial-material>`、 :ref:`rgba<tendon-spatial-rgba>` 属性を反映します。 :ref:`range<tendon-spatial-range>` または :ref:`springlength<tendon-spatial-springlength>` が :at-val:`[0 X]` の形式（正の X）を持つ、非駆動の2点テンドンには、特別な可視化が用いられます。このようなテンドンはケーブルのように振る舞い、伸びた時のみ力を加えます。したがって、伸びていない時は、長さ X のカテナリ曲線として描画されます。`this example model <https://github.com/google-deepmind/mujoco/blob/main/test/engine/testdata/catenary.xml>`__ の右側のクリップを参照してください。

.. _tendon-spatial-name:

:at:`name`: :at-val:`string, optional`
   テンドンの名前。

.. _tendon-spatial-class:

:at:`class`: :at-val:`string, optional`
   未指定の属性を設定するデフォルトクラス。

.. _tendon-spatial-group:

:at:`group`: :at-val:`int, "0"`
   テンドンが所属する整数グループ。この属性はカスタムタグに使用できます。また、ビジュアライザーがテンドンのグループ全体のレンダリングを有効化/無効化するために使用されます。

.. _tendon-spatial-limited:

:at:`limited`: :at-val:`[false, true, auto], "auto"`
   この属性が "true" の場合、以下の range 属性で定義される長さ制限が、制約ソルバーによって課されます。この属性が "auto" で、かつ :ref:`compiler <compiler>` で :at:`autolimits` が設定されている場合、range が定義されていれば長さ制限が有効化されます。

.. _tendon-spatial-actuatorfrclimited:

:at:`actuatorfrclimited`: :at-val:`[false, true, auto], "auto"`
   この属性は、テンドンに作用するアクチュエータの力をクランプするかどうかを指定します。詳細は :ref:`CForceRange` を参照してください。この属性は :ref:`actuatorfrcrange<tendon-spatial-actuatorfrcrange>` 属性と相互作用します。この属性が "false" の場合、アクチュエータ力のクランプは無効化されます。"true" の場合、アクチュエータ力のクランプが有効化されます。この属性が "auto" で、かつ :ref:`compiler <compiler>` で :at:`autolimits` が設定されている場合、 :at:`actuatorfrcrange` が定義されていればアクチュエータ力のクランプが有効化されます。

.. _tendon-spatial-range:

:at:`range`: :at-val:`real(2), "0 0"`
   許容されるテンドン長の範囲。この属性を設定する際に :at:`limited` を指定しないのはエラーです。ただし、 :ref:`compiler <compiler>` で :at:`autolimits` が設定されている場合は除きます。

.. _tendon-spatial-actuatorfrcrange:

:at:`actuatorfrcrange`: :at-val:`real(2), "0 0"`
   このテンドンに作用する全アクチュエータ力をクランプする範囲。詳細は :ref:`CForceRange` を参照してください。コンパイラは、下限が非正、上限が非負であることを期待します。|br| この属性を設定する際に :at:`actuatorfrclimited` を指定しないのは、 :at:`compiler-autolimits` が "false" の場合はエラーです。

.. _tendon-spatial-solreflimit:

.. _tendon-spatial-solimplimit:

:at:`solreflimit`, :at:`solimplimit`
   テンドン制限をシミュレートするための制約ソルバーパラメータ。 :ref:`CSolver` を参照してください。

.. _tendon-spatial-solreffriction:

.. _tendon-spatial-solimpfriction:

:at:`solreffriction`, :at:`solimpfriction`
   テンドンの乾燥摩擦をシミュレートするための制約ソルバーパラメータ。 :ref:`CSolver` を参照してください。

.. _tendon-spatial-margin:

:at:`margin`: :at-val:`real, "0"`
   テンドン長と指定範囲のいずれかの制限との差の絶対値がこのマージンを下回ると、制限制約がアクティブになります。接触と同様に、マージンパラメータは、範囲制限とテンドン長との差から減算されます。結果として得られる制約距離は、制約がアクティブな時は常に負です。この量は、 :ref:`CSolver` で説明されているように、距離の関数として制約インピーダンスを計算するために使用されます。

.. _tendon-spatial-frictionloss:

:at:`frictionloss`: :at-val:`real, "0"`
   乾燥摩擦による摩擦損失。摩擦損失を有効化するには、この属性を正の値に設定します。

.. _tendon-spatial-width:

:at:`width`: :at-val:`real, "0.003"`
   レンダリングに使用される、空間テンドンの断面積の半径。ジオム障害物の周りを巻き付くテンドンの部分は、幅を減少させてレンダリングされます。

.. _tendon-spatial-material:

:at:`material`: :at-val:`string, optional`
   テンドンの外観を設定するために使用されるマテリアル。

.. _tendon-spatial-rgba:

:at:`rgba`: :at-val:`real(4), "0.5 0.5 0.5 1"`
   テンドンの色と透明度。この値が内部デフォルトと異なる場合、対応するマテリアル特性を上書きします。 :at:`material` が未指定で、 :at:`rgba` がデフォルト値を持つ場合、長さが制限を超える制限付きテンドンは、 :ref:`制約インピーダンス<soParameters>` :math:`d` の値を使用して、デフォルト色と :ref:`rgba/constraint<visual-rgba-constraint>` を混合した色に変更されます。

.. _tendon-spatial-springlength:

:at:`springlength`: :at-val:`real(2), "-1 -1"`
   バネの静止位置。1つまたは2つの値を取ることができます。1つの値が与えられた場合、静止時のテンドンの長さに対応します。``-1`` の場合、テンドンの静止長は ``mjModel.qpos0`` のモデル参照コンフィギュレーションから決定されます。|br| デフォルト値 ``-1`` は自動長さ計算を呼び出しますが、これは非負の長さしか持てない :ref:`spatial<tendon-spatial>` テンドンを念頭に設計されました。 :ref:`fixed<tendon-fixed>` テンドンの :at:`springlength` を ``-1`` に設定するには、``-0.99999`` のような近い値を使用してください。|br| 2つの非減少値が与えられた場合、それらは `不感帯 <https://en.wikipedia.org/wiki/Deadband>`_ 範囲を定義します。テンドン長が2つの値の間にある場合、力は0です。この範囲外にある場合、力は通常のバネのように振る舞い、静止点は最も近い :at:`springlength` 値に対応します。不感帯を使用して、制限が制約ではなくバネによって強制されるテンドンを定義できます。

.. _tendon-spatial-stiffness:

:at:`stiffness`: :at-val:`real, "0"`
   剛性係数。正の値は、テンドンに沿って作用するバネ力（位置に線形）を生成します。

.. _tendon-spatial-damping:

:at:`damping`: :at-val:`real, "0"`
   減衰係数。正の値は、テンドンに沿って作用する減衰力（速度に線形）を生成します。オイラー法で陰的に積分されるジョイント減衰とは異なり、テンドン減衰は陰的に積分されないため、可能であればジョイント減衰を使用すべきです。

.. image:: images/XMLreference/tendon_armature.gif
   :width: 30%
   :align: right
   :class: only-light
   :target: https://github.com/google-deepmind/mujoco/blob/main/test/engine/testdata/core_smooth/ten_armature_1_compare.xml
.. image:: images/XMLreference/tendon_armature_dark.gif
   :width: 30%
   :align: right
   :class: only-dark
   :target: https://github.com/google-deepmind/mujoco/blob/main/test/engine/testdata/core_smooth/ten_armature_1_compare.xml

.. _tendon-spatial-armature:

:at:`armature`: :at-val:`real, "0"`
   テンドン長の変化に伴う慣性。この属性を正の値 :math:`m` に設定すると、運動エネルギー項 :math:`\frac{1}{2}mv^2` が追加されます。ここで :math:`v` はテンドン速度です。テンドン慣性は、回転要素を含む線形アクチュエータの :ref:`armature<body-joint-armature>` 慣性や、線形油圧アクチュエータ内の流体の慣性運動をモデル化する際に最も有用です。図では、(*左*) :ref:`armature<body-joint-armature>` を持つ回転ジョイントとスライダージョイントで「テンドン」を実装し、 :ref:`connect<equality-connect>` 制約でワールドに接続した3自由度システムと、(*右*) アーマチュア慣性を持つテンドンを使った等価な1自由度モデルを比較しています。ジョイント :ref:`armature<body-joint-armature>` と同様に、この追加慣性はテンドン長の変化にのみ関連し、固定長で移動するテンドンの動力学には影響しません。テンドンヤコビアン :math:`J` は位置依存であるため、テンドンアーマチュアは追加のバイアス力項 :math:`c = m J \dot{J}^T \dot{q}` を生じさせます。

.. _tendon-spatial-user:

:at:`user`: :at-val:`real(nuser_tendon), "0 0 ..."`
   :ref:`CUser` を参照してください。


.. _spatial-site:

:el-prefix:`spatial/` |-| **site** |m|
''''''''''''''''''''''''''''''''''''''

この属性は、テンドン経路が通過する必要があるサイトを指定します。サイトはボディに剛的に取り付けられることを思い出してください。

.. _spatial-site-site:

:at:`site`: :at-val:`string, required`
   テンドンが通過する必要があるサイトの名前。


.. _spatial-geom:

:el-prefix:`spatial/` |-| **geom** |m|
''''''''''''''''''''''''''''''''''''''

この要素は、テンドン経路の障害物として機能するジオムを指定します。最小長さ経路がジオムに触れない場合、効果はありません。そうでない場合、経路はジオムの表面を巻き付きます。巻き付きは解析的に計算されるため、ここで許可されるジオムタイプを球と円筒に制限しています。後者は、テンドン巻き付きの目的では無限長として扱われます。サイドサイトが定義され、その位置がジオム内部にある場合、テンドンは周囲を通るのではなく、ジオムの内部を通過するよう制約されます。

.. _spatial-geom-geom:

:at:`geom`: :at-val:`string, required`
   テンドン経路の障害物として機能するジオムの名前。ここでは球と円筒のジオムのみを参照できます。

.. _spatial-geom-sidesite:

:at:`sidesite`: :at-val:`string, optional`
   モデルコンフィギュレーションの変化に応じてテンドン経路がジオムの一方の側から他方へスナップするのを防ぐため、ユーザーはジオムの優先「側」を定義できます。実行時には、指定されたサイトに近い巻き付きが自動的に選択されます。サイドサイトの指定は、実際にはしばしば必要です。サイドサイトがジオム内部にある場合、テンドンはジオムの内部を通過するよう制約されます。


.. _spatial-pulley:

:el-prefix:`spatial/` |-| **pulley** |m|
''''''''''''''''''''''''''''''''''''''''

この要素は、テンドン経路に新しい分岐を開始します。分岐は空間的に接続されている必要はありません。Computation章の :ref:`Actuation model <geActuation>` セクションで説明されている伝達と同様に、シミュレーションに影響する量は、テンドン長とその関節位置に対する勾配です。空間テンドンが複数の分岐を持つ場合、各分岐の長さは、分岐を開始したプーリー要素の divisor 属性で除算され、合計されて全体のテンドン長を得ます。これが、分岐間の空間的関係がシミュレーションに関係しない理由です。上記の `tendon.xml <_static/tendon.xml>`__ 例は、プーリーの使用を示しています。

.. _spatial-pulley-divisor:

:at:`divisor`: :at-val:`real, required`
   プーリー要素によって開始されたテンドン分岐の長さは、ここで指定された値で除算されます。1つの分岐を2つの並列分岐に分割する物理的なプーリーの場合、共通の分岐は divisor 値 1 を持ち、プーリーに続く2つの分岐は divisor 値 2 を持ちます。そのうちの1つが別のプーリーによってさらに分割される場合、各新しい分岐は divisor 値 4 を持ちます。MJCF では各分岐がプーリーで始まるため、1つの物理的なプーリーは2つの MJCF プーリーでモデル化されることに注意してください。テンドン経路にプーリー要素が含まれていない場合、最初で唯一の分岐は divisor 値 1 を持ちます。


.. _tendon-fixed:

:el-prefix:`tendon/` |-| **fixed** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

この要素は、長さが関節位置の線形結合として定義される抽象テンドンを作成します。シミュレーションに必要な量は、テンドン長とその勾配のみであることを思い出してください。したがって、関節位置の任意のスカラー関数を定義し、それを「テンドン」と呼んで MuJoCo に接続できます。現在のところ、このような関数は固定線形結合のみです。固定テンドンの属性は、空間テンドンの属性のサブセットであり、上記と同じ意味を持ちます。

.. _tendon-fixed-name:

.. _tendon-fixed-class:

.. _tendon-fixed-group:

.. _tendon-fixed-limited:

.. _tendon-fixed-actuatorfrclimited:

.. _tendon-fixed-range:

.. _tendon-fixed-actuatorfrcrange:

.. _tendon-fixed-solreflimit:

.. _tendon-fixed-solimplimit:

.. _tendon-fixed-solreffriction:

.. _tendon-fixed-solimpfriction:

.. _tendon-fixed-frictionloss:

.. _tendon-fixed-margin:

.. _tendon-fixed-springlength:

.. _tendon-fixed-stiffness:

.. _tendon-fixed-damping:

.. _tendon-fixed-armature:

.. _tendon-fixed-user:

.. |tendon/fixed attrib list| replace::
   :at:`name`, :at:`class`, :at:`group`, :at:`limited`, :at:`range`, :at:`solreflimit`, :at:`solimplimit`,
   :at:`solreffriction`, :at:`solimpfriction`, :at:`frictionloss`, :at:`margin`, :at:`springlength`, :at:`stiffness`,
   :at:`damping`, :at:`user`

|tendon/fixed attrib list|
   :ref:`spatial <tendon-spatial>` 要素と同じです。


.. _fixed-joint:

:el-prefix:`fixed/` |-| **joint** |m|
'''''''''''''''''''''''''''''''''''''

この要素は、固定テンドン長の計算にジョイントを追加します。含まれる各ジョイントの位置または角度に、対応する coef 値が乗算され、合計されてテンドン長が得られます。

.. _fixed-joint-joint:

:at:`joint`: :at-val:`string, required`
   固定テンドンに追加するジョイントの名前。ここではスカラージョイント（slide と hinge）のみを参照できます。

.. _fixed-joint-coef:

:at:`coef`: :at-val:`real, required`
   指定されたジョイントの位置または角度に乗算するスカラー係数。


.. _actuator:

**actuator** |m|
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

これは、アクチュエータ定義のグルーピング要素です。Computation章の MuJoCo の :ref:`Actuation model <geActuation>` の議論、およびこの章で前述した :ref:`Actuator shortcuts <CActShortcuts>` を思い出してください。以下のすべてのアクチュエータ関連要素の最初の13個の属性は同じであるため、 :el:`general` アクチュエータの下で一度だけ文書化します。


.. _actuator-general:

:el-prefix:`actuator/` |-| **general** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

この要素は、汎用アクチュエータを作成し、すべてのアクチュエータコンポーネントへの完全なアクセスを提供し、ユーザーがそれらを独立して指定できるようにします。


.. _actuator-general-name:

:at:`name`: :at-val:`string, optional`
   要素名。 :ref:`CName` を参照してください。

.. _actuator-general-class:

:at:`class`: :at-val:`string, optional`
   アクティブなデフォルトクラス。 :ref:`CDefault` を参照してください。

.. _actuator-general-group:

:at:`group`: :at-val:`int, "0"`
   アクチュエータが所属する整数グループ。この属性はカスタムタグに使用できます。また、ビジュアライザーがアクチュエータのグループ全体のレンダリングを有効化/無効化するために使用されます。

.. _actuator-general-ctrllimited:

:at:`ctrllimited`: :at-val:`[false, true, auto], "auto"`
   true の場合、このアクチュエータへの制御入力は、実行時に自動的に :at:`ctrlrange` にクランプされます。false の場合、制御入力のクランプは無効化されます。"auto" で、かつ :ref:`compiler <compiler>` で :at:`autolimits` が設定されている場合、この属性を明示的に "true" に設定せずに :at:`ctrlrange` が定義されていれば、制御クランプが自動的に true に設定されます。制御入力のクランプは、 :ref:`option/flag <option-flag>` の :at:`clampctrl` 属性でグローバルに無効化することもできることに注意してください。

.. _actuator-general-forcelimited:

:at:`forcelimited`: :at-val:`[false, true, auto], "auto"`
   true の場合、このアクチュエータの力出力は、実行時に自動的に :at:`forcerange` にクランプされます。false の場合、力のクランプは無効化されます。"auto" で、かつ :ref:`compiler <compiler>` で :at:`autolimits` が設定されている場合、この属性を明示的に "true" に設定せずに :at:`forcerange` が定義されていれば、力のクランプが自動的に true に設定されます。

.. _actuator-general-actlimited:

:at:`actlimited`: :at-val:`[false, true, auto], "auto"`
   true の場合、このアクチュエータに関連する内部状態（活性化）は、実行時に自動的に :at:`actrange` にクランプされます。false の場合、活性化のクランプは無効化されます。"auto" で、かつ :ref:`compiler <compiler>` で :at:`autolimits` が設定されている場合、この属性を明示的に "true" に設定せずに :at:`actrange` が定義されていれば、活性化のクランプが自動的に true に設定されます。詳細は :ref:`Activation clamping <CActRange>` セクションを参照してください。

.. _actuator-general-ctrlrange:

:at:`ctrlrange`: :at-val:`real(2), "0 0"`
   制御入力をクランプする範囲。最初の値は2番目の値より小さい必要があります。|br| この属性を設定する際に :at:`ctrllimited` を指定しないのは、 :ref:`compiler <compiler>` で :at:`autolimits` が "false" の場合はエラーです。

.. _actuator-general-forcerange:

:at:`forcerange`: :at-val:`real(2), "0 0"`
   力出力をクランプする範囲。最初の値は2番目の値以下でなければなりません。|br| この属性を設定する際に :at:`forcelimited` を指定しないのは、 :ref:`compiler <compiler>` で :at:`autolimits` が "false" の場合はエラーです。

.. _actuator-general-actrange:

:at:`actrange`: :at-val:`real(2), "0 0"`
   活性化状態をクランプする範囲。最初の値は2番目の値以下でなければなりません。詳細は :ref:`Activation clamping <CActRange>` セクションを参照してください。|br| この属性を設定する際に :at:`actlimited` を指定しないのは、 :ref:`compiler <compiler>` で :at:`autolimits` が "false" の場合はエラーです。

.. _actuator-general-lengthrange:

:at:`lengthrange`: :at-val:`real(2), "0 0"`
   アクチュエータの伝達の実現可能な長さの範囲。 :ref:`Length Range <CLengthRange>` を参照してください。

.. _actuator-general-gear:

:at:`gear`: :at-val:`real(6), "1 0 0 0 0 0"`
   この属性は、すべての伝達タイプに対して、アクチュエータの長さ（したがってモーメントアーム、速度、力）をスケールします。これは、力生成メカニズムのゲインとは異なります。ゲインは力出力のみをスケールし、長さ、モーメントアーム、速度には影響しないためです。スカラー伝達を持つアクチュエータの場合、このベクトルの最初の要素のみが使用されます。残りの要素は、この属性が3D力とトルク軸を指定するために使用される、joint、jointinparent、site 伝達に必要です。

.. _actuator-general-cranklength:

:at:`cranklength`: :at-val:`real, "0"`
   スライダークランク伝達タイプにのみ使用されます。コネクティングロッドの長さを指定します。コンパイラは、スライダークランク伝達が存在する場合、この値が正であることを期待します。

.. _actuator-general-joint:

:at:`joint`: :at-val:`string, optional`
   これと次の4つの属性は、アクチュエータ伝達のタイプを決定します。これらはすべてオプションであり、そのうちの正確に1つを指定する必要があります。この属性が指定された場合、アクチュエータは与えられたジョイントに作用します。**hinge** および **slide** ジョイントの場合、アクチュエータ長はジョイント位置/角度に :at:`gear` の最初の要素を乗算したものに等しくなります。**ball** ジョイントの場合、gear の最初の3つの要素が、アクチュエータがトルクを生成する子フレーム内の3D回転軸を定義します。アクチュエータ長は、このギア軸とジョイントクォータニオンの角度軸表現との内積として定義され、 :at:`gear` が正規化されている場合はラジアン単位です（一般的には :at:`gear` のノルムでスケールされます）。 :math:`\pi` を超える全回転の後、長さは :math:`-\pi` にラップし、逆も同様です。したがって、ボールジョイント用の :el:`position` サーボは、一般的にこのラッピングを防ぐより厳しい制限を使用すべきです。**free** ジョイントの場合、gear はワールドフレーム内の3D並進軸と子フレーム内の3D回転軸を定義します。アクチュエータは、指定された軸に対して力とトルクを生成します。フリージョイントのアクチュエータ長はゼロとして定義されます（したがって、ポジションサーボと一緒に使用すべきではありません）。

.. _actuator-general-jointinparent:

:at:`jointinparent`: :at-val:`string, optional`
   joint と同一ですが、ball と free ジョイントの場合、gear で与えられる3D回転軸が、子フレームではなく親フレーム（free ジョイントの場合はワールドフレーム）で定義される点が異なります。

.. _actuator-general-site:

:at:`site`: :at-val:`string, optional`
   この伝達は、サイトに力とトルクを適用できます。gear ベクトルは、3D並進軸に続いて3D回転軸を定義します。両方ともサイトのフレームで定義されます。これは、ジェットやプロペラをモデル化するために使用できます。効果はフリージョイントを駆動することに似ており、アクチュエータ長は、 :at:`refsite` が定義されていない限りゼロとして定義されます（以下を参照）。上記の :at:`joint` および :at:`jointinparent` 伝達との1つの違いは、ここではアクチュエータがジョイントではなくサイトに作用することですが、この違いは、サイトが浮遊ボディのフレーム原点で定義されている場合は消失します。もう1つの違いは、site 伝達の場合、並進軸と回転軸の両方がローカル座標で定義されることです。対照的に、 :at:`joint` の場合は並進がグローバルで回転がローカルであり、 :at:`jointinparent` の場合は並進と回転の両方がグローバルです。

..  youtube:: s-0JHanqV1A
    :align: right
    :height: 150px

.. _actuator-general-refsite:

:at:`refsite`: :at-val:`string, optional`
   :at:`site` 伝達を使用する場合、 :at:`refsite` のフレームに対する並進と回転を測定します。この場合、アクチュエータは長さを *持ち*、 :el:`position` アクチュエータを使用してエンドエフェクタを直接制御できます。`refsite.xml <https://github.com/google-deepmind/mujoco/tree/main/test/engine/testdata/actuation/refsite.xml>`__ 例モデルを参照してください。上記と同様に、長さは :at:`gear` ベクトルとフレーム差の内積です。したがって ``gear="0 1 0 0 0 0"`` は「 :at:`refsite` フレーム内の :at:`site` の Y オフセット」を意味し、``gear="0 0 0 0 0 1"`` は「 :at:`refsite` フレーム内の :at:`site` の Z 回転」を意味します。正規化された :at:`gear` ベクトルを使用し、最初の3つ *または* 最後の3つの要素のみに非ゼロを持つことを推奨します。これにより、アクチュエータ長は長さ単位またはラジアンのいずれかになります。ボールジョイントと同様に（上記の :at:`joint` を参照）、 :math:`\pi` を超える全角度の回転はラップアラウンドするため、より厳しい制限を推奨します。

.. _actuator-general-body:

:at:`body`: :at-val:`string, optional`
   この伝達は、接触法線の方向に接触点で線形力を適用できます。接触のセットは、指定された :at:`body` に属するすべてのものです。これは、ヤモリや昆虫の足のような自然な能動接着メカニズムをモデル化するために使用できます。アクチュエータ長は再びゼロとして定義されます。詳細については、以下の :ref:`adhesion<actuator-adhesion>` ショートカットを参照してください。

.. _actuator-general-tendon:

:at:`tendon`: :at-val:`string, optional`
   指定された場合、アクチュエータは与えられたテンドンに作用します。アクチュエータ長は、テンドン長にギア比を乗算したものに等しくなります。空間テンドンと固定テンドンの両方を使用できます。

.. _actuator-general-cranksite:

:at:`cranksite`: :at-val:`string, optional`
   指定された場合、アクチュエータは、アクチュエータによって暗黙的に決定されるスライダークランクメカニズムに作用します（つまり、別個のモデル要素ではありません）。指定されたサイトは、クランクとコネクティングロッドを結合するピンに対応します。アクチュエータ長は、スライダークランクメカニズムの位置にギア比を乗算したものに等しくなります。

.. _actuator-general-slidersite:

:at:`slidersite`: :at-val:`string, required for slider-crank transmission`
   スライダークランク伝達タイプにのみ使用されます。指定されたサイトは、スライダーとコネクティングロッドを結合するピンです。スライダーは、slidersite フレームの z 軸に沿って移動します。したがって、サイトは、キネマティックツリーで定義される際に必要に応じて向きを設定する必要があります。その向きは、アクチュエータ定義では変更できません。

.. _actuator-general-user:

:at:`user`: :at-val:`real(nuser_actuator), "0 ... 0"`
   :ref:`CUser` を参照してください。

.. _actuator-general-actdim:

:at:`actdim`: :at-val:`real, "-1"`
   活性化状態の次元。デフォルト値 ``-1`` は、コンパイラに :at:`dyntype` に従って次元を設定するよう指示します。``1`` より大きい値は、ユーザー定義の活性化ダイナミクスに対してのみ許可されます。ネイティブタイプは次元 0 または 1 のみを必要とするためです。活性化次元が 1 より大きい場合、*最後の要素* が力の生成に使用されます。

.. _actuator-general-dyntype:

:at:`dyntype`: :at-val:`[none, integrator, filter, filterexact, muscle, user], "none"`
   アクチュエータの活性化ダイナミクスタイプ。利用可能なダイナミクスタイプは、 :ref:`Actuation model <geActuation>` セクションで既に説明されています。やや異なる表記（mjModel と mjData フィールドに対応）でその説明を繰り返すと、次のようになります:

   =========== ======================================
   キーワード  説明
   =========== ======================================
   none        内部状態なし
   integrator  act_dot = ctrl
   filter      act_dot = (ctrl - act) / dynprm[0]
   filterexact filter と同様だが厳密積分を使用
   muscle      act_dot = mju_muscleDynamics(...)
   user        act_dot = mjcb_act_dyn(...)
   =========== ======================================

.. _actuator-general-gaintype:

:at:`gaintype`: :at-val:`[fixed, affine, muscle, user], "fixed"`
   ゲインとバイアスは一緒に、力生成メカニズムの出力を決定します。これは現在、アフィンであると仮定されています。 :ref:`Actuation model <geActuation>` で既に説明したように、一般的な式は次のとおりです:
   scalar_force = gain_term \* (act or ctrl) + bias_term。
   この式は、存在する場合は活性化状態を使用し、そうでない場合は制御を使用します。キーワードには次の意味があります:

   ======= ===============================
   キーワード 説明
   ======= ===============================
   fixed   gain_term = gainprm[0]
   affine  gain_term = gain_prm[0] + gain_prm[1]*length + gain_prm[2]*velocity
   muscle  gain_term = mju_muscleGain(...)
   user    gain_term = mjcb_act_gain(...)
   ======= ===============================

.. _actuator-general-biastype:

:at:`biastype`: :at-val:`[none, affine, muscle, user], "none"`
   キーワードには次の意味があります:

   ======= ================================================================
   キーワード 説明
   ======= ================================================================
   none    bias_term = 0
   affine  bias_term = biasprm[0] + biasprm[1]*length + biasprm[2]*velocity
   muscle  bias_term = mju_muscleBias(...)
   user    bias_term = mjcb_act_bias(...)
   ======= ================================================================

.. _actuator-general-dynprm:

:at:`dynprm`: :at-val:`real(10), "1 0 ... 0"`
   活性化ダイナミクスパラメータ。組み込みの活性化タイプ（muscle を除く）は最初のパラメータのみを使用しますが、ユーザーコールバックがより複雑なモデルを実装する場合に備えて、追加のパラメータを提供しています。この配列の長さはパーサーによって強制されないため、ユーザーは必要な数のパラメータを入力できます。これらのデフォルトは、muscle アクチュエータとは互換性がありません。以下の :ref:`muscle <actuator-muscle>` を参照してください。

.. _actuator-general-gainprm:

:at:`gainprm`: :at-val:`real(10), "1 0 ... 0"`
   ゲインパラメータ。組み込みのゲインタイプ（muscle を除く）は最初のパラメータのみを使用しますが、ユーザーコールバックがより複雑なモデルを実装する場合に備えて、追加のパラメータを提供しています。この配列の長さはパーサーによって強制されないため、ユーザーは必要な数のパラメータを入力できます。これらのデフォルトは、muscle アクチュエータとは互換性がありません。以下の :ref:`muscle <actuator-muscle>` を参照してください。

.. _actuator-general-biasprm:

:at:`biasprm`: :at-val:`real(10), "0 ... 0"`
   バイアスパラメータ。アフィンバイアスタイプは3つのパラメータを使用します。この配列の長さはパーサーによって強制されないため、ユーザーは必要な数のパラメータを入力できます。これらのデフォルトは、muscle アクチュエータとは互換性がありません。以下の :ref:`muscle <actuator-muscle>` を参照してください。

.. _actuator-general-actearly:

:at:`actearly`: :at-val:`[false, true], "false"`
   true の場合、力の計算は現在の活性化変数の値ではなく、次の値を使用します。このフラグを設定すると、制御と加速度の間の遅延が1タイムステップ分削減されます。

.. _actuator-motor:

:el-prefix:`actuator/` |-| **motor** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

この要素と次の3つの要素は、前述の :ref:`アクチュエータショートカット <CActShortcuts>` です。このようなショートカットが検出されると、パーサーは :el:`general` アクチュエータを作成し、dynprm、gainprm、biasprm属性を上記の内部デフォルト値に設定します（デフォルト設定に関わらず）。その後、ショートカットに応じてdyntype、gaintype、biastypeを調整し、カスタム属性（共通属性以外）を解析して、ここで説明するように通常の属性（つまり :el:`general` アクチュエータタイプの属性）に変換します。

この要素は直接駆動アクチュエータを作成します。基礎となる :el:`general` 属性は以下のように設定されます：

========= ======= ========= =======
Attribute Setting Attribute Setting
========= ======= ========= =======
dyntype   none    dynprm    1 0 0
gaintype  fixed   gainprm   1 0 0
biastype  none    biasprm   0 0 0
========= ======= ========= =======


この要素にはカスタム属性はありません。共通属性のみがあります：

.. _actuator-motor-name:

.. _actuator-motor-class:

.. _actuator-motor-group:

.. _actuator-motor-ctrllimited:

.. _actuator-motor-forcelimited:

.. _actuator-motor-ctrlrange:

.. _actuator-motor-forcerange:

.. _actuator-motor-lengthrange:

.. _actuator-motor-gear:

.. _actuator-motor-cranklength:

.. _actuator-motor-joint:

.. _actuator-motor-jointinparent:

.. _actuator-motor-tendon:

.. _actuator-motor-cranksite:

.. _actuator-motor-slidersite:

.. _actuator-motor-site:

.. _actuator-motor-refsite:

.. _actuator-motor-user:


.. |actuator/motor attrib list| replace::
   :at:`name`, :at:`class`, :at:`group`, :at:`ctrllimited`, :at:`forcelimited`, :at:`ctrlrange`, :at:`forcerange`,
   :at:`lengthrange`, :at:`gear`, :at:`cranklength`, :at:`joint`, :at:`jointinparent`, :at:`tendon`, :at:`cranksite`,
   :at:`slidersite`, :at:`site`, :at:`refsite`, :at:`user`

|actuator/motor attrib list|
   actuator/ :ref:`general <actuator-general>` と同様。


.. _actuator-position:

:el-prefix:`actuator/` |-| **position** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

この要素は、オプションの一次フィルタを持つ位置サーボを作成します。基礎となる :el:`general` 属性は以下のように設定されます：

========= =================== ========= =============
Attribute Setting             Attribute Setting
========= =================== ========= =============
dyntype   none or filterexact dynprm    timeconst 0 0
gaintype  fixed               gainprm   kp 0 0
biastype  affine              biasprm   0 -kp -kv
========= =================== ========= =============


この要素は、共通属性に加えて1つのカスタム属性を持ちます：

.. _actuator-position-name:

.. _actuator-position-class:

.. _actuator-position-group:

.. _actuator-position-ctrllimited:

.. _actuator-position-forcelimited:

.. _actuator-position-ctrlrange:

.. _actuator-position-forcerange:

.. _actuator-position-lengthrange:

.. _actuator-position-gear:

.. _actuator-position-cranklength:

.. _actuator-position-joint:

.. _actuator-position-jointinparent:

.. _actuator-position-tendon:

.. _actuator-position-cranksite:

.. _actuator-position-slidersite:

.. _actuator-position-site:

.. _actuator-position-refsite:

.. _actuator-position-user:

.. |actuator/position attrib list| replace::
   :at:`name`, :at:`class`, :at:`group`, :at:`ctrllimited`, :at:`forcelimited`, :at:`ctrlrange`, :at:`forcerange`,
   :at:`lengthrange`, :at:`gear`, :at:`cranklength`, :at:`joint`, :at:`jointinparent`, :at:`tendon`, :at:`cranksite`,
   :at:`slidersite`, :at:`site`, :at:`refsite`, :at:`user`

|actuator/position attrib list|
   actuator/ :ref:`general <actuator-general>` と同様。

.. _actuator-position-kp:

:at:`kp`: :at-val:`real, "1"`
   位置フィードバックゲイン。

.. _actuator-position-kv:

:at:`kv`: :at-val:`real, "0"`
   アクチュエータによる減衰。
   この属性を使用する場合は、implicitfastまたはimplicit :ref:`積分器<geIntegration>` を使用することをお勧めします。

.. _actuator-position-dampratio:

:at:`dampratio`: :at-val:`real, "0"`
   減衰比の単位を使用した、アクチュエータによる減衰。
   この属性は :at:`kv` と排他的であり、意味は似ていますが、力/速度の単位ではなく、単位は :math:`2 \sqrt{k_p \cdot m}` であり、調和振動子の `減衰比 <https://en.wikipedia.org/wiki/Damping#Damping_ratio_definition>`__ に対応します。
   値1は *臨界減衰* の振動子に対応し、しばしば望ましい挙動を生成します。
   1より小さいまたは大きい値は、それぞれ劣減衰および過減衰の振動に対応します。
   質量 :math:`m` は、参照姿勢 ``mjModel.qpos0`` で計算され、ジョイントの :ref:`armature <body-joint-armature>` を考慮に入れます。
   ただし、影響を受けるジョイントの受動的な :ref:`damping <body-joint-damping>` または :ref:`frictionloss <body-joint-frictionloss>` は考慮されません。それらが無視できない場合、望ましい動きを実現するために1より小さい :at:`dampratio` 値が必要になる可能性があります。
   この属性を使用する場合は、implicitfastまたはimplicit :ref:`積分器<geIntegration>` を使用することをお勧めします。

.. _actuator-position-timeconst:

:at:`timeconst`: :at-val:`real, "0"`
   オプションの一次フィルタの時定数。0より大きい場合、アクチュエータは :at:`filterexact` :ref:`ダイナミクスタイプ<actuator-general-dyntype>` を使用し、0（デフォルト）の場合はフィルタは使用されません。


.. _actuator-position-inheritrange:

:at:`inheritrange`: :at-val:`real, "0"`
   伝達ターゲットの :at:`range` に合わせてアクチュエータの :at:`ctrlrange` を自動的に設定します。デフォルト値は「無効」を意味します。正の値 :at-val:`X` は、ターゲット範囲の中点を中心に :at-val:`X` でスケーリングされた :at:`ctrlrange` を設定します。たとえば、ターゲットジョイントの :at:`range` が :at-val:`[0, 1]` の場合、値 :at-val:`1.0` は :at:`ctrlrange` を :at-val:`[0, 1]` に設定します。値 :at-val:`0.8` および :at-val:`1.2` は :at:`ctrlrange` を :at-val:`[0.1, 0.9]` および :at-val:`[-0.1, 1.1]` にそれぞれ設定します。1より小さい値は制限に到達しないようにするために有用です。1より大きい値は制限での制御権限を維持する（制限に押し付けることができる）ために有用です。この属性は :at:`ctrlrange` と排他的であり、 :at:`range` が定義されたジョイントおよびテンドン伝達でのみ使用できます。 :at:`inheritrange` は :ref:`position<actuator-position>` 属性と :ref:`デフォルトクラス<default-position-inheritrange>` の両方で使用できますが、保存されたXMLは常にアクチュエータで明示的な :at:`ctrlrange` に変換することに注意してください。

.. _actuator-velocity:

:el-prefix:`actuator/` |-| **velocity** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

この要素は速度サーボを作成します。PDコントローラーを作成するには、2つのアクチュエータ（位置サーボと速度サーボ）を定義する必要があることに注意してください。これは、MuJoCoアクチュエータがSISO（Single Input Single Output）であるのに対し、PDコントローラーは2つの制御入力（参照位置と参照速度）を取るためです。
このアクチュエータを使用する場合は、implicitfastまたはimplicit :ref:`積分器<geIntegration>` を使用することをお勧めします。
基礎となる :el:`general` 属性は以下のように設定されます：

========= ======= ========= =======
Attribute Setting Attribute Setting
========= ======= ========= =======
dyntype   none    dynprm    1 0 0
gaintype  fixed   gainprm   kv 0 0
biastype  affine  biasprm   0 0 -kv
========= ======= ========= =======

この要素は、共通属性に加えて1つのカスタム属性を持ちます：

.. _actuator-velocity-name:

.. _actuator-velocity-class:

.. _actuator-velocity-group:

.. _actuator-velocity-ctrllimited:

.. _actuator-velocity-forcelimited:

.. _actuator-velocity-ctrlrange:

.. _actuator-velocity-forcerange:

.. _actuator-velocity-lengthrange:

.. _actuator-velocity-gear:

.. _actuator-velocity-cranklength:

.. _actuator-velocity-joint:

.. _actuator-velocity-jointinparent:

.. _actuator-velocity-tendon:

.. _actuator-velocity-cranksite:

.. _actuator-velocity-slidersite:

.. _actuator-velocity-site:

.. _actuator-velocity-refsite:

.. _actuator-velocity-user:

.. |actuator/velocity attrib list| replace::
   :at:`name`, :at:`class`, :at:`group`, :at:`ctrllimited`, :at:`forcelimited`, :at:`ctrlrange`, :at:`forcerange`,
   :at:`lengthrange`, :at:`gear`, :at:`cranklength`, :at:`joint`, :at:`jointinparent`, :at:`tendon`, :at:`cranksite`,
   :at:`slidersite`, :at:`site`, :at:`refsite`, :at:`user`

|actuator/velocity attrib list|
   actuator/ :ref:`general <actuator-general>` と同様。

.. _actuator-velocity-kv:

:at:`kv`: :at-val:`real, "1"`
   速度フィードバックゲイン。


.. _actuator-intvelocity:

:el-prefix:`actuator/` |-| **intvelocity** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

この要素は、積分速度サーボを作成します。詳細については、モデリングの章の :ref:`活性化クランプ <CActRange>` セクションを参照してください。基礎となる :el:`general` 属性は以下のように設定されます：

==========   =========== ========= =========
Attribute    Setting     Attribute Setting
==========   =========== ========= =========
dyntype      integrator  dynprm    1 0 0
gaintype     fixed       gainprm   kp 0 0
biastype     affine      biasprm   0 -kp -kv
actlimited   true
==========   =========== ========= =========

この要素は、共通属性に加えて1つのカスタム属性を持ちます：

.. _actuator-intvelocity-name:

.. _actuator-intvelocity-class:

.. _actuator-intvelocity-group:

.. _actuator-intvelocity-ctrllimited:

.. _actuator-intvelocity-forcelimited:

.. _actuator-intvelocity-ctrlrange:

.. _actuator-intvelocity-forcerange:

.. _actuator-intvelocity-actrange:

.. _actuator-intvelocity-lengthrange:

.. _actuator-intvelocity-gear:

.. _actuator-intvelocity-cranklength:

.. _actuator-intvelocity-joint:

.. _actuator-intvelocity-jointinparent:

.. _actuator-intvelocity-tendon:

.. _actuator-intvelocity-cranksite:

.. _actuator-intvelocity-slidersite:

.. _actuator-intvelocity-site:

.. _actuator-intvelocity-refsite:

.. _actuator-intvelocity-user:

.. |actuator/intvelocity attrib list| replace::
   :at:`name`, :at:`class`, :at:`group`, :at:`ctrllimited`, :at:`forcelimited`, :at:`ctrlrange`, :at:`forcerange`,
   :at:`actrange`, :at:`lengthrange`, :at:`gear`, :at:`cranklength`, :at:`joint`, :at:`jointinparent`, :at:`tendon`,
   :at:`cranksite`, :at:`slidersite`, :at:`site`, :at:`refsite`, :at:`user`

|actuator/intvelocity attrib list|
   actuator/ :ref:`general <actuator-general>` と同様。

.. _actuator-intvelocity-kp:

:at:`kp`: :at-val:`real, "1"`
   位置フィードバックゲイン。

.. _actuator-intvelocity-kv:

:at:`kv`: :at-val:`real, "0"`
   アクチュエータによる減衰。
   この属性を使用する場合は、implicitfastまたはimplicit :ref:`積分器<geIntegration>` を使用することをお勧めします。

.. _actuator-intvelocity-dampratio:

:at:`dampratio`: :at-val:`real, "0"`
   :ref:`position/dampratio<actuator-position-dampratio>` を参照してください。

.. _actuator-intvelocity-inheritrange:

:at:`inheritrange`: :at-val:`real, "0"`
   :ref:`position/inheritrange<actuator-position-inheritrange>` と同一ですが、 :at:`ctrlrange`（速度の意味を持つ）ではなく :at:`actrange`（伝達ターゲットと同じ長さの意味を持つ）を設定します。

.. _actuator-damper:

:el-prefix:`actuator/` |-| **damper** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

この要素は、速度と制御の両方に比例する力を生成するアクティブダンパーです：``F = - kv * velocity * control``。ここで ``kv`` は非負でなければなりません。 :at:`ctrlrange` は必須であり、非負でなければなりません。
このアクチュエータを使用する場合は、implicitfastまたはimplicit :ref:`積分器<geIntegration>` を使用することをお勧めします。
基礎となる :el:`general` 属性は以下のように設定されます：

=========== ======= ========= =======
Attribute   Setting Attribute Setting
=========== ======= ========= =======
dyntype     none    dynprm    1 0 0
gaintype    affine  gainprm   0 0 -kv
biastype    none    biasprm   0 0 0
ctrllimited true
=========== ======= ========= =======


この要素は、共通属性に加えて1つのカスタム属性を持ちます：

.. _actuator-damper-name:

.. _actuator-damper-class:

.. _actuator-damper-group:

.. _actuator-damper-ctrllimited:

.. _actuator-damper-forcelimited:

.. _actuator-damper-ctrlrange:

.. _actuator-damper-forcerange:

.. _actuator-damper-lengthrange:

.. _actuator-damper-gear:

.. _actuator-damper-cranklength:

.. _actuator-damper-joint:

.. _actuator-damper-jointinparent:

.. _actuator-damper-tendon:

.. _actuator-damper-cranksite:

.. _actuator-damper-slidersite:

.. _actuator-damper-site:

.. _actuator-damper-refsite:

.. _actuator-damper-user:

.. |actuator/damper attrib list| replace:: :at:`name`, :at:`class`, :at:`group`, :at:`ctrllimited`, :at:`forcelimited`,
   :at:`ctrlrange`, :at:`forcerange`, :at:`lengthrange`, :at:`gear`, :at:`cranklength`, :at:`joint`,
   :at:`jointinparent`, :at:`tendon`, :at:`cranksite`, :at:`slidersite`, :at:`site`, :at:`refsite`, :at:`user`

|actuator/damper attrib list|
   actuator/ :ref:`general <actuator-general>` と同様。

.. _actuator-damper-kv:

:at:`kv`: :at-val:`real, "1"`
   速度フィードバックゲイン。


.. _actuator-cylinder:

:el-prefix:`actuator/` |-| **cylinder** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

この要素は、空気圧または油圧シリンダのモデリングに適しています。基礎となる :el:`general` 属性は以下のように設定されます：

========= ======= ========= =============
Attribute Setting Attribute Setting
========= ======= ========= =============
dyntype   filter  dynprm    timeconst 0 0
gaintype  fixed   gainprm   area 0 0
biastype  affine  biasprm   bias(3)
========= ======= ========= =============


この要素は、共通属性に加えて4つのカスタム属性を持ちます：

.. _actuator-cylinder-name:

.. _actuator-cylinder-class:

.. _actuator-cylinder-group:

.. _actuator-cylinder-ctrllimited:

.. _actuator-cylinder-forcelimited:

.. _actuator-cylinder-ctrlrange:

.. _actuator-cylinder-forcerange:

.. _actuator-cylinder-lengthrange:

.. _actuator-cylinder-gear:

.. _actuator-cylinder-cranklength:

.. _actuator-cylinder-joint:

.. _actuator-cylinder-jointinparent:

.. _actuator-cylinder-tendon:

.. _actuator-cylinder-cranksite:

.. _actuator-cylinder-slidersite:

.. _actuator-cylinder-site:

.. _actuator-cylinder-refsite:

.. _actuator-cylinder-user:

.. |actuator/cylinder attrib list| replace::
   :at:`name`, :at:`class`, :at:`group`, :at:`ctrllimited`, :at:`forcelimited`, :at:`ctrlrange`, :at:`forcerange`,
   :at:`lengthrange`, :at:`gear`, :at:`cranklength`, :at:`joint`, :at:`jointinparent`, :at:`tendon`, :at:`cranksite`,
   :at:`slidersite`, :at:`site`, :at:`refsite`, :at:`user`

|actuator/cylinder attrib list|
   actuator/ :ref:`general <actuator-general>` と同様。

.. _actuator-cylinder-timeconst:

:at:`timeconst`: :at-val:`real, "1"`
   活性化ダイナミクスの時定数。

.. _actuator-cylinder-area:

:at:`area`: :at-val:`real, "1"`
   シリンダの面積。これはアクチュエータゲインとして内部的に使用されます。

.. _actuator-cylinder-diameter:

:at:`diameter`: :at-val:`real, optional`
   面積の代わりにユーザーは直径を指定できます。両方が指定されている場合は、直径が優先されます。

.. _actuator-cylinder-bias:

:at:`bias`: :at-val:`real(3), "0 0 0"`
   バイアスパラメータ。内部的にbiasprmにコピーされます。


.. _actuator-muscle:

:el-prefix:`actuator/` |-| **muscle** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

この要素は、 :ref:`マッスルアクチュエータ <CMuscle>` セクションで説明されているように、マッスルアクチュエータのモデリングに使用されます。基礎となる :el:`general` 属性は以下のように設定されます：

========= ======= ========= ======================================================
Attribute Setting Attribute Setting
========= ======= ========= ======================================================
dyntype   muscle  dynprm    timeconst(2) tausmooth
gaintype  muscle  gainprm   range(2), force, scale, lmin, lmax, vmax, fpmax, fvmax
biastype  muscle  biasprm   gainprmと同じ
========= ======= ========= ======================================================


この要素は、共通属性に加えて9つのカスタム属性を持ちます：

.. _actuator-muscle-name:

.. _actuator-muscle-class:

.. _actuator-muscle-group:

.. _actuator-muscle-ctrllimited:

.. _actuator-muscle-forcelimited:

.. _actuator-muscle-ctrlrange:

.. _actuator-muscle-forcerange:

.. _actuator-muscle-lengthrange:

.. _actuator-muscle-gear:

.. _actuator-muscle-cranklength:

.. _actuator-muscle-joint:

.. _actuator-muscle-jointinparent:

.. _actuator-muscle-tendon:

.. _actuator-muscle-cranksite:

.. _actuator-muscle-slidersite:

.. _actuator-muscle-user:


.. |actuator/muscle attrib list| replace::
   :at:`name`, :at:`class`, :at:`group`, :at:`ctrllimited`, :at:`forcelimited`, :at:`ctrlrange`, :at:`forcerange`,
   :at:`lengthrange`, :at:`gear`, :at:`cranklength`, :at:`joint`, :at:`jointinparent`, :at:`tendon`, :at:`cranksite`,
   :at:`slidersite`, :at:`user`

|actuator/muscle attrib list|
   actuator/ :ref:`general <actuator-general>` と同様。

.. _actuator-muscle-timeconst:

:at:`timeconst`: :at-val:`real(2), "0.01 0.04"`
   活性化および非活性化ダイナミクスの時定数。

.. _actuator-muscle-tausmooth:

:at:`tausmooth`: :at-val:`real, "0"`
   活性化と非活性化の時定数の間のスムーズな遷移の幅。ctrlの単位で、非負でなければなりません。

.. _actuator-muscle-range:

:at:`range`: :at-val:`real(2), "0.75 1.05"`
   L0の単位でのマッスルの動作長さ範囲。

.. _actuator-muscle-force:

:at:`force`: :at-val:`real, "-1"`
   静止時のピークアクティブ力。この値が負の場合、ピーク力は以下のscale属性を使用して自動的に決定されます。

.. _actuator-muscle-scale:

:at:`scale`: :at-val:`real, "200"`
   force属性が負の場合、マッスルのピークアクティブ力はこの値をmjModel.actuator_acc0で割った値に設定されます。後者は、qpos0でのアクチュエータの伝達に対する単位力によって引き起こされる関節空間加速度ベクトルのノルムです。言い換えると、スケーリングは、より多くの重量を引っ張るマッスルに対してより高いピーク力を生成します。

.. _actuator-muscle-lmin:

:at:`lmin`: :at-val:`real, "0.5"`
   L0の単位での正規化FLV曲線の下部位置範囲。

.. _actuator-muscle-lmax:

:at:`lmax`: :at-val:`real, "1.6"`
   L0の単位での正規化FLV曲線の上部位置範囲。

.. _actuator-muscle-vmax:

:at:`vmax`: :at-val:`real, "1.5"`
   マッスル力がゼロに低下する短縮速度。L0/秒の単位。

.. _actuator-muscle-fpmax:

:at:`fpmax`: :at-val:`real, "1.3"`
   lmaxで生成される受動力。ピーク静止力に対する相対値。

.. _actuator-muscle-fvmax:

:at:`fvmax`: :at-val:`real, "1.2"`
   飽和伸長速度で生成されるアクティブ力。ピーク静止力に対する相対値。


.. _actuator-adhesion:

:el-prefix:`actuator/` |-| **adhesion** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

..  youtube:: BcHZ5BFeTmU
    :align: right
    :height: 150px

この要素は、接触における法線方向に力を注入するアクティブ接着アクチュエータを定義します。図の動画を参照してください。動画に示されているモデルは `こちら <https://github.com/google-deepmind/mujoco/tree/main/model/adhesion>`_ にあり、インライン注釈が含まれています。伝達ターゲットは :el:`body` であり、接着力はこのボディに属するジオムを含むすべての接触に注入されます。力は複数の接触間で均等に分割されます。 :at:`gap` 属性が使用されない場合、このアクチュエータはアクティブな接触を必要とし、距離で力を適用することはできません。これは、産業用真空グリッパーよりも、ヤモリや昆虫の足のアクティブ接着に近いものです。「距離での吸引」を有効にするために、 :at:`margin` でボディのジオムを「膨張」させ、 :at:`gap` 浸透距離後にのみ接触をアクティブにする対応する :at:`gap` を追加します。これにより、接触が検出されるがアクティブではないジオムの周りにレイヤーが作成され、接着力の適用に使用できます。上の動画では、このような非アクティブな接触は青色で、アクティブな接触はオレンジ色です。
接着アクチュエータの長さは常に0です。 :at:`ctrlrange` は必須であり、非負でなければなりません（反発力は許可されていません）。基礎となる :el:`general` 属性は以下のように設定されます：

=========== ======= =========== ========
Attribute   Setting Attribute   Setting
=========== ======= =========== ========
dyntype     none    dynprm      1 0 0
gaintype    fixed   gainprm     gain 0 0
biastype    none    biasprm     0 0 0
trntype     body    ctrllimited true
=========== ======= =========== ========

この要素は、共通属性のサブセットと2つのカスタム属性を持ちます。

.. _actuator-adhesion-name:

.. _actuator-adhesion-class:

.. _actuator-adhesion-group:

.. _actuator-adhesion-forcelimited:

.. _actuator-adhesion-ctrlrange:

.. _actuator-adhesion-forcerange:

.. _actuator-adhesion-user:

.. |actuator/adhesion attrib list| replace:: :at:`name`, :at:`class`, :at:`group`,
   :at:`forcelimited`, :at:`ctrlrange`, :at:`forcerange`, :at:`user`

|actuator/adhesion attrib list|
   actuator/ :ref:`general <actuator-general>` と同様。

.. _actuator-adhesion-body:

:at:`body`: :at-val:`string, required`
   アクチュエータは、このボディのジオムを含むすべての接触に作用します。

.. _actuator-adhesion-gain:

:at:`gain`: :at-val:`real, "1"`
   接着アクチュエータのゲイン。力の単位。アクチュエータによって適用される総接着力は、制御値にゲインを掛けたものです。この力は、ターゲットボディに属するジオムを含むすべての接触間で均等に分配されます。


.. _actuator-plugin:

:el-prefix:`actuator/` |-| **plugin** |?|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

このアクチュエータを :ref:`エンジンプラグイン<exPlugin>` に関連付けます。 :at:`plugin` または :at:`instance` のいずれかが必須です。

.. _actuator-plugin-plugin:

:at:`plugin`: :at-val:`string, optional`
   プラグイン識別子。暗黙的なプラグインのインスタンス化に使用されます。

.. _actuator-plugin-instance:

:at:`instance`: :at-val:`string, optional`
   インスタンス名。明示的なプラグインのインスタンス化に使用されます。

.. _actuator-plugin-dyntype:

:at:`dyntype`: :at-val:`[none, integrator, filter, filterexact, muscle, user], "none"`
   アクチュエータの活性化ダイナミクスタイプ。利用可能なダイナミクスタイプは、 :ref:`アクチュエーションモデル <geActuation>` セクションですでに説明されています。 :ref:`dyntype<actuator-general-dyntype>` が "none" でない場合、アクチュエータに活性化変数が追加されます。この変数は、プラグインによって計算された活性化状態の後に追加されます（ :ref:`アクチュエータプラグイン活性化<exActuatorAct>` を参照）。

.. _actuator-plugin-actrange:

:at:`actrange`: :at-val:`real(2), "0 0"`
   このアクチュエータのdyntypeに関連する活性化状態をクランプするための範囲。制限はプラグインによって計算された活性化には適用されません。最初の値は2番目の値以下でなければなりません。
   詳細については、 :ref:`活性化クランプ <CActRange>` セクションを参照してください。

.. _actuator-plugin-name:

.. _actuator-plugin-class:

.. _actuator-plugin-group:

.. _actuator-plugin-actlimited:

.. _actuator-plugin-ctrllimited:

.. _actuator-plugin-forcelimited:

.. _actuator-plugin-ctrlrange:

.. _actuator-plugin-forcerange:

.. _actuator-plugin-lengthrange:

.. _actuator-plugin-gear:

.. _actuator-plugin-cranklength:

.. _actuator-plugin-joint:

.. _actuator-plugin-jointinparent:

.. _actuator-plugin-site:

.. _actuator-plugin-tendon:

.. _actuator-plugin-cranksite:

.. _actuator-plugin-slidersite:

.. _actuator-plugin-user:

.. _actuator-plugin-actdim:

.. _actuator-plugin-dynprm:

.. _actuator-plugin-actearly:

.. |actuator/plugin attrib list| replace:: :at:`name`, :at:`class`, :at:`group`, :at:`actlimited`, :at:`ctrllimited`,
   :at:`forcelimited`, :at:`ctrlrange`, :at:`forcerange`, :at:`lengthrange`, :at:`gear`, :at:`cranklength`,
   :at:`joint`, :at:`jointinparent`, :at:`site`, :at:`tendon`, :at:`cranksite`, :at:`slidersite`, :at:`user`,
   :at:`actdim`, :at:`dynprm`, :at:`actearly`

|actuator/plugin attrib list|
   actuator/ :ref:`general <actuator-general>` と同様。


.. _sensor:

**sensor** |m|
~~~~~~~~~~~~~~

これはセンサー定義のグループ化要素です。属性はありません。すべてのセンサーの出力は、mjData.sensordataフィールドに連結されます。このフィールドのサイズはmjModel.nsensordataです。このデータは内部計算では使用されません。

以下の要素で作成されたセンサーに加えて、トップレベル関数 :ref:`mj_step` は、mjData.cacc、mjData.cfrc_int、mjData.crfc_extの量を計算します。これらはボディの加速度と相互作用力に対応します。これらの量の一部は、特定のセンサー（力、加速度など）の出力を計算するために使用されますが、そのようなセンサーがモデルで定義されていない場合でも、これらの量自体はユーザーにとって興味のある「特徴」となる可能性があります。


.. _sensor-touch:

:el-prefix:`sensor/` |-| **touch** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

この要素はタッチセンサーを作成します。アクティブセンサーゾーンはサイトによって定義されます。接触点がサイトの体積内に入り、サイトと同じボディに接続されたジオムを含む場合、対応する接触力がセンサーの読み取り値に含まれます。接触点がセンサーゾーンの外側にあるが、法線レイがセンサーゾーンと交差する場合も含まれます。この再投影機能は、それがないと（柔らかい接触のために）接触点がセンサーゾーンの背面から出てしまい、誤った力の読み取り値を引き起こす可能性があるため必要です。このセンサーの出力は非負のスカラーです。これは、含まれるすべての接触からの（スカラー）法線力を合計することによって計算されます。

.. _sensor-touch-name:

.. _sensor-touch-noise:

.. _sensor-touch-cutoff:

.. _sensor-touch-user:

:at:`name`, :at:`noise`, :at:`cutoff`, :at:`user`
   :ref:`CSensor` を参照してください。

.. _sensor-touch-site:

:at:`site`: :at-val:`string, required`
   アクティブセンサーゾーンを定義するサイト。


.. _sensor-accelerometer:

:el-prefix:`sensor/` |-| **accelerometer** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

この要素は3軸加速度計を作成します。センサーはサイトに取り付けられており、サイトフレームと同じ位置と向きを持ちます。このセンサーは3つの数値を出力します。これらはサイトの線形加速度（重力を含む）をローカル座標で表したものです。

モデルにこのセンサーが存在すると、センサー計算中に :ref:`mj_rnePostConstraint` の呼び出しがトリガーされます。

.. _sensor-accelerometer-name:

.. _sensor-accelerometer-noise:

.. _sensor-accelerometer-cutoff:

.. _sensor-accelerometer-user:

:at:`name`, :at:`noise`, :at:`cutoff`, :at:`user`
   :ref:`CSensor` を参照してください。

.. _sensor-accelerometer-site:

:at:`site`: :at-val:`string, required`
   センサーが取り付けられているサイト。加速度計はサイトローカルフレームに中心を置き、整列されます。


.. _sensor-velocimeter:

:el-prefix:`sensor/` |-| **velocimeter** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

この要素は3軸速度計を作成します。センサーはサイトに取り付けられており、サイトフレームと同じ位置と向きを持ちます。このセンサーは3つの数値を出力します。これらはサイトの線形速度をローカル座標で表したものです。

.. _sensor-velocimeter-name:

.. _sensor-velocimeter-noise:

.. _sensor-velocimeter-cutoff:

.. _sensor-velocimeter-user:

:at:`name`, :at:`noise`, :at:`cutoff`, :at:`user`
   :ref:`CSensor` を参照してください。

.. _sensor-velocimeter-site:

:at:`site`: :at-val:`string, required`
   センサーが取り付けられているサイト。速度計はサイトローカルフレームに中心を置き、整列されます。


.. _sensor-gyro:

:el-prefix:`sensor/` |-| **gyro** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

この要素は3軸ジャイロスコープを作成します。センサーはサイトに取り付けられており、サイトフレームと同じ位置と向きを持ちます。このセンサーは3つの数値を出力します。これらはサイトの角速度をローカル座標で表したものです。このセンサーは、慣性計測ユニット（IMU）をシミュレートするために、同じサイトに取り付けられた :ref:`加速度計 <sensor-accelerometer>` と組み合わせて使用されることがよくあります。

.. _sensor-gyro-name:

.. _sensor-gyro-noise:

.. _sensor-gyro-cutoff:

.. _sensor-gyro-user:

:at:`name`, :at:`noise`, :at:`cutoff`, :at:`user`
   :ref:`CSensor` を参照してください。

.. _sensor-gyro-site:

:at:`site`: :at-val:`string, required`
   センサーが取り付けられているサイト。ジャイロスコープはサイトローカルフレームに中心を置き、整列されます。


.. _sensor-force:

:el-prefix:`sensor/` |-| **force** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

この要素は3軸力センサーを作成します。センサーは3つの数値を出力します。これらは子ボディと親ボディの間の相互作用力を、センサーを定義するサイトフレームで表現したものです。規約として、サイトは子ボディに接続され、力は子から親に向かって指します。ここでの計算は、接触だけでなく、外部の摂動を含む、システムに作用するすべての力を考慮します。このセンサーを使用するには、多くの場合、親に溶接されたダミーのボディ（つまり、ジョイント要素を持たない）を作成する必要があります。

モデルにこのセンサーが存在すると、センサー計算中に :ref:`mj_rnePostConstraint` の呼び出しがトリガーされます。

.. _sensor-force-name:

.. _sensor-force-noise:

.. _sensor-force-cutoff:

.. _sensor-force-user:

:at:`name`, :at:`noise`, :at:`cutoff`, :at:`user`
   :ref:`CSensor` を参照してください。

.. _sensor-force-site:

:at:`site`: :at-val:`string, required`
   センサーが取り付けられているサイト。測定される相互作用力は、サイトが定義されているボディとその親ボディの間にあり、子から親に向かって指します。モデル化されている物理センサーは、もちろん親ボディに接続されている可能性があります。その場合、センサーデータの符号は逆になります。各ボディには一意の親がありますが、複数の子を持つことができるため、このセンサーをペアの親ボディではなく子ボディを通じて定義します。


.. _sensor-torque:

:el-prefix:`sensor/` |-| **torque** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

この要素は3軸トルクセンサーを作成します。これは上記の :ref:`力 <sensor-force>` センサーに似ていますが、力ではなくトルクを測定します。

モデルにこのセンサーが存在すると、センサー計算中に :ref:`mj_rnePostConstraint` の呼び出しがトリガーされます。

.. _sensor-torque-name:

.. _sensor-torque-noise:

.. _sensor-torque-cutoff:

.. _sensor-torque-user:

:at:`name`, :at:`noise`, :at:`cutoff`, :at:`user`
   :ref:`CSensor` を参照してください。

.. _sensor-torque-site:

:at:`site`: :at-val:`string, required`
   センサーが取り付けられているサイト。測定される相互作用トルクは、サイトが定義されているボディとその親ボディの間にあります。


.. _sensor-magnetometer:

:el-prefix:`sensor/` |-| **magnetometer** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

この要素は磁力計を作成します。センサーサイトの位置での磁束を、センサーサイトフレームで表現して測定します。出力は3次元ベクトルです。

.. _sensor-magnetometer-name:

.. _sensor-magnetometer-noise:

.. _sensor-magnetometer-cutoff:

.. _sensor-magnetometer-user:

:at:`name`, :at:`noise`, :at:`cutoff`, :at:`user`
   :ref:`CSensor` を参照してください。

.. _sensor-magnetometer-site:

:at:`site`: :at-val:`string, required`
   センサーが取り付けられているサイト。

.. _sensor-rangefinder:

:el-prefix:`sensor/` |-| **rangefinder** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

この要素は測距計を作成します。

- :ref:`サイト<sensor-rangefinder-site>` に関連付けられている場合、サイトの正のZ軸で定義されるレイに沿って、最も近いジオム表面までの距離を測定します。
- :ref:`カメラ<sensor-rangefinder-camera>` に関連付けられている場合、カメラ画像の各ピクセルに対して1つの距離測定値を出力します。カメラはそのフレームの :ref:`負のZ軸<body-camera>` を向いていることに注意してください。この場合の測定数は、カメラの幅と高さの :ref:`解像度<body-camera-resolution>` の積に等しくなります。

.. image:: images/XMLreference/rfcamera.png
   :width: 45%
   :align: right
   :target: https://github.com/google-deepmind/mujoco/blob/main/test/engine/testdata/sensor/rfcamera.xml

レイがジオム表面と交差しない場合、センサー出力は-1です。レイの原点がジオムの内部にある場合でも、表面は検出されます。センサーサイト/カメラと同じボディに接続されているジオムは除外されます。不可視なジオム、つまりrgba（またはそのマテリアルのrgba）がalpha=0であるジオムも除外されます。ただし、ジオムグループを無効にすることでビジュアライザーで不可視にされたジオムは除外されないことに注意してください。これは、センサー計算がビジュアライザーから独立しているためです。

右の画像（クリックして可視化されているモデルを表示）は、透視カメラと正投影カメラに接続された2つの測距計センサーを示しており、錐台が可視化されています。両方のカメラの解像度は4x4で、それぞれ16本のレイがあります。測距計センサーは :at:`data` = :at-val:`"dist point normal"` を報告します（以下を参照）。したがって、レイ（線）、交差点（球）、表面法線（矢印）が表示されます。

.. _sensor-rangefinder-data:

:at:`data`: :at-val:`[dist, dir, origin, point, normal, depth], "dist"`
   デフォルトでは、測距計は上記のように距離測定値を出力します。ただし、出力データフィールドのセットを指定することもできます。 :at:`data` 属性には **複数の順次データタイプ** を含めることができます。ただし、上記のリストの相対順序を維持する必要があります。たとえば、 :at:`data` = :at-val:`"dist point normal"` はレイごとに7つの数値を返しますが、 :at:`data` = :at-val:`"point origin"` はエラーです。 :at-val:`origin` は :at-val:`point` の前に来る必要があるためです。

   - :at-val:`dist` **real(1)**: レイの原点から最も近いジオム表面までの距離。表面に当たらなかった場合は-1。このデータタイプが含まれている場合、レイは線として可視化されます。
   - :at-val:`dir` **real(3)**: レイの正規化された方向。表面に当たらなかった場合は(0, 0, 0)。
   - :at-val:`origin` **real(3)**: レイが発する点（グローバルフレーム）。サイトと透視カメラの場合、これはサイト/カメラのxposです。ただし、正投影カメラの場合、レイの原点は画像平面に沿って空間的に分布しています。
   - :at-val:`point` **real(3)**: レイが最も近いジオム表面と交差する点（グローバルフレーム）。表面に当たらなかった場合は(0, 0, 0)。このデータタイプが含まれている場合、交差点は球として可視化されます。
   - :at-val:`normal`: **real(3)**: レイが交差する点でのジオム表面法線（グローバルフレーム）。表面に当たらなかった場合は(0, 0, 0)。法線は常にジオム表面の外側を向いており、レイの原点には関係ありません。このデータタイプが :at-val:`dist` または :at-val:`point` とともに含まれている場合、法線は交差点で矢印として可視化されます。
   - :at-val:`depth`: **real(1)**: ヒット点からカメラ平面までの距離。表面に当たらなかった場合は-1。この深度セマンティクスは、コンピュータグラフィックスの意味での深度画像に対応します。

.. _sensor-rangefinder-name:

.. _sensor-rangefinder-noise:

.. _sensor-rangefinder-cutoff:

.. _sensor-rangefinder-user:

:at:`name`, :at:`noise`, :at:`cutoff`, :at:`user`
   :ref:`CSensor` を参照してください。

.. _sensor-rangefinder-site:

:at:`site`: :at-val:`string, optional`
   センサーが取り付けられているサイト。

.. _sensor-rangefinder-camera:

:at:`camera`: :at-val:`string, optional`
   センサーが取り付けられているカメラ。

.. _sensor-camprojection:

:el-prefix:`sensor/` |-| **camprojection** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

この要素はカメラ投影センサーを作成します。ターゲットサイトの位置を、カメラ画像にピクセル座標で投影します。ピクセルの原点(0, 0)は左上隅にあります。値はクリッピングされないため、カメラ画像の外側に落ちるターゲットは、ピクセル範囲の制限より上または下の値を取ります。さらに、カメラの背後にある点も画像に投影されるため、必要に応じてそのような点をフィルタリングすることはユーザー次第です。これは、カメラを基準フレームとして :ref:`framepos<sensor-framepos>` センサーを使用して実行できます。z座標の負/正の値は、カメラ平面の前/後ろの位置を示します。

.. _sensor-camprojection-site:

:at:`site`: :at-val:`string, required`
   カメラ画像に投影されるサイト。

.. _sensor-camprojection-camera:

:at:`camera`: :at-val:`string, required`
   投影に使用されるカメラ。その :ref:`解像度<body-camera-resolution>` 属性は正である必要があります。

.. _sensor-camprojection-name:

.. _sensor-camprojection-noise:

.. _sensor-camprojection-cutoff:

.. _sensor-camprojection-user:

:at:`name`, :at:`noise`, :at:`cutoff`, :at:`user`
   :ref:`CSensor` を参照してください。

.. _sensor-jointpos:

:el-prefix:`sensor/` |-| **jointpos** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

これ以降のセンサー要素は、センサー固有の計算を含みません。代わりに、すでに計算されている量を配列mjData.sensordataにコピーします。この要素はジョイントの位置または角度センサーを作成します。これはスカラージョイント（スライドまたはヒンジ）に接続できます。その出力はスカラーです。

.. _sensor-jointpos-name:

.. _sensor-jointpos-noise:

.. _sensor-jointpos-cutoff:

.. _sensor-jointpos-user:

:at:`name`, :at:`noise`, :at:`cutoff`, :at:`user`
   :ref:`CSensor` を参照してください。

.. _sensor-jointpos-joint:

:at:`joint`: :at-val:`string, required`
   位置または角度がセンシングされるジョイント。ここではスカラージョイントのみを参照できます。センサー出力はmjData.qposからコピーされます。


.. _sensor-jointvel:

:el-prefix:`sensor/` |-| **jointvel** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

この要素はジョイントの速度センサーを作成します。これはスカラージョイント（スライドまたはヒンジ）に接続できます。その出力はスカラーです。

.. _sensor-jointvel-name:

.. _sensor-jointvel-noise:

.. _sensor-jointvel-cutoff:

.. _sensor-jointvel-user:

:at:`name`, :at:`noise`, :at:`cutoff`, :at:`user`
   :ref:`CSensor` を参照してください。

.. _sensor-jointvel-joint:

:at:`joint`: :at-val:`string, required`
   速度がセンシングされるジョイント。ここではスカラージョイントのみを参照できます。センサー出力はmjData.qvelからコピーされます。


.. _sensor-tendonpos:

:el-prefix:`sensor/` |-| **tendonpos** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

この要素はテンドンの長さセンサーを作成します。空間テンドンと固定テンドンの両方に接続できます。その出力はスカラーです。

.. _sensor-tendonpos-name:

.. _sensor-tendonpos-noise:

.. _sensor-tendonpos-cutoff:

.. _sensor-tendonpos-user:

:at:`name`, :at:`noise`, :at:`cutoff`, :at:`user`
   :ref:`CSensor` を参照してください。

.. _sensor-tendonpos-tendon:

:at:`tendon`: :at-val:`string, required`
   長さがセンシングされるテンドン。センサー出力はmjData.ten_lengthからコピーされます。


.. _sensor-tendonvel:

:el-prefix:`sensor/` |-| **tendonvel** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

この要素はテンドンの速度センサーを作成します。空間テンドンと固定テンドンの両方に接続できます。その出力はスカラーです。

.. _sensor-tendonvel-name:

.. _sensor-tendonvel-noise:

.. _sensor-tendonvel-cutoff:

.. _sensor-tendonvel-user:

:at:`name`, :at:`noise`, :at:`cutoff`, :at:`user`
   :ref:`CSensor` を参照してください。

.. _sensor-tendonvel-tendon:

:at:`tendon`: :at-val:`string, required`
   速度がセンシングされるテンドン。センサー出力はmjData.ten_velocityからコピーされます。


.. _sensor-actuatorpos:

:el-prefix:`sensor/` |-| **actuatorpos** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

この要素はアクチュエータの長さセンサーを作成します。各アクチュエータには長さを持つ伝達機構があることを思い出してください。このセンサーは任意のアクチュエータに接続できます。その出力はスカラーです。

.. _sensor-actuatorpos-name:

.. _sensor-actuatorpos-noise:

.. _sensor-actuatorpos-cutoff:

.. _sensor-actuatorpos-user:

:at:`name`, :at:`noise`, :at:`cutoff`, :at:`user`
   :ref:`CSensor` を参照してください。

.. _sensor-actuatorpos-actuator:

:at:`actuator`: :at-val:`string, required`
   伝達機構の長さがセンシングされるアクチュエータ。センサー出力はmjData.actuator_lengthからコピーされます。


.. _sensor-actuatorvel:

:el-prefix:`sensor/` |-| **actuatorvel** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

この要素はアクチュエータの速度センサーを作成します。このセンサーは任意のアクチュエータに接続できます。その出力はスカラーです。

.. _sensor-actuatorvel-name:

.. _sensor-actuatorvel-noise:

.. _sensor-actuatorvel-cutoff:

.. _sensor-actuatorvel-user:

:at:`name`, :at:`noise`, :at:`cutoff`, :at:`user`
   :ref:`CSensor` を参照してください。

.. _sensor-actuatorvel-actuator:

:at:`actuator`: :at-val:`string, required`
   伝達機構の速度がセンシングされるアクチュエータ。センサー出力はmjData.actuator_velocityからコピーされます。


.. _sensor-actuatorfrc:

:el-prefix:`sensor/` |-| **actuatorfrc** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

この要素はアクチュエータの力センサーを作成します。センシングされる量は、スカラーのアクチュエータ力であり、アクチュエータによって寄与される一般化力（後者は、スカラー力と伝達機構によって決定されるモーメントアームのベクトルの積）ではありません。このセンサーは任意のアクチュエータに接続できます。その出力はスカラーです。

.. _sensor-actuatorfrc-name:

.. _sensor-actuatorfrc-noise:

.. _sensor-actuatorfrc-cutoff:

.. _sensor-actuatorfrc-user:

:at:`name`, :at:`noise`, :at:`cutoff`, :at:`user`
   :ref:`CSensor` を参照してください。

.. _sensor-actuatorfrc-actuator:

:at:`actuator`: :at-val:`string, required`
   スカラー力出力がセンシングされるアクチュエータ。センサー出力はmjData.actuator_forceからコピーされます。


.. _sensor-jointactuatorfrc:

:el-prefix:`sensor/` |-| **jointactuatorfrc** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

この要素はジョイントで測定されるアクチュエータ力センサーを作成します。センシングされる量は、すべてのアクチュエータによって単一のスカラージョイント（ヒンジまたはスライダー）に寄与される一般化力です。ジョイントの :ref:`actuatorgravcomp<body-joint-actuatorgravcomp>` 属性が「true」の場合、このセンサーは重力補償力（ジョイントに直接追加され、 :ref:`actuatorfrc<sensor-actuatorfrc>` センサーには *登録されない* ）による寄与も測定します。このタイプのセンサーは、複数のアクチュエータが単一のジョイントに作用する場合、または単一のアクチュエータが複数のジョイントに作用する場合に重要です。詳細については、 :ref:`CForceRange` を参照してください。


.. _sensor-jointactuatorfrc-name:

.. _sensor-jointactuatorfrc-noise:

.. _sensor-jointactuatorfrc-cutoff:

.. _sensor-jointactuatorfrc-user:

:at:`name`, :at:`noise`, :at:`cutoff`, :at:`user`
   :ref:`CSensor` を参照してください。

.. _sensor-jointactuatorfrc-joint:

:at:`joint`: :at-val:`string, required`
   アクチュエータ力がセンシングされるジョイント。センサー出力は ``mjData.qfrc_actuator`` からコピーされます。


.. _sensor-tendonactuatorfrc:

:el-prefix:`sensor/` |-| **tendonactuatorfrc** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

この要素はテンドンで測定されるアクチュエータ力センサーを作成します。センシングされる量は、すべてのアクチュエータによって単一のテンドンに寄与される合力です。このタイプのセンサーは、複数のアクチュエータが単一のテンドンに作用する場合に重要です。詳細については、 :ref:`CForceRange` を参照してください。


.. _sensor-tendonactuatorfrc-name:

.. _sensor-tendonactuatorfrc-noise:

.. _sensor-tendonactuatorfrc-cutoff:

.. _sensor-tendonactuatorfrc-user:

:at:`name`, :at:`noise`, :at:`cutoff`, :at:`user`
   :ref:`CSensor` を参照してください。

.. _sensor-tendonactuatorfrc-tendon:

:at:`tendon`: :at-val:`string, required`
   アクチュエータ力がセンシングされるテンドン。


.. _sensor-ballquat:

:el-prefix:`sensor/` |-| **ballquat** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

この要素はボールジョイントのクォータニオンセンサーを作成します。単位クォータニオンに対応する4つの数値を出力します。

.. _sensor-ballquat-name:

.. _sensor-ballquat-noise:

.. _sensor-ballquat-cutoff:

.. _sensor-ballquat-user:

:at:`name`, :at:`noise`, :at:`cutoff`, :at:`user`
   :ref:`CSensor` を参照してください。

.. _sensor-ballquat-joint:

:at:`joint`: :at-val:`string, required`
   クォータニオンがセンシングされるボールジョイント。センサー出力はmjData.qposからコピーされます。


.. _sensor-ballangvel:

:el-prefix:`sensor/` |-| **ballangvel** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

この要素はボールジョイントの角速度センサーを作成します。ジョイントの角速度に対応する3つの数値を出力します。そのベクトルのノルムはrad/sでの回転速度であり、方向は回転が行われる軸です。

.. _sensor-ballangvel-name:

.. _sensor-ballangvel-noise:

.. _sensor-ballangvel-cutoff:

.. _sensor-ballangvel-user:

:at:`name`, :at:`noise`, :at:`cutoff`, :at:`user`
   :ref:`CSensor` を参照してください。

.. _sensor-ballangvel-joint:

:at:`joint`: :at-val:`string, required`
   角速度がセンシングされるボールジョイント。センサー出力はmjData.qvelからコピーされます。


.. _sensor-jointlimitpos:

:el-prefix:`sensor/` |-| **jointlimitpos** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

この要素は位置のためのジョイントリミットセンサーを作成します。

.. _sensor-jointlimitpos-name:

.. _sensor-jointlimitpos-noise:

.. _sensor-jointlimitpos-cutoff:

.. _sensor-jointlimitpos-user:

:at:`name`, :at:`noise`, :at:`cutoff`, :at:`user`
   :ref:`CSensor` を参照してください。

.. _sensor-jointlimitpos-joint:

:at:`joint`: :at-val:`string, required`
   リミットがセンシングされるジョイント。センサー出力は、対応するリミット制約に対してmjData.efc_pos - mjData.efc_marginに等しくなります。リミットが違反されている場合、どちら側のリミットが違反されていても、結果は負になることに注意してください。リミットの両側が同時に違反されている場合、最初のコンポーネントのみが返されます。違反がない場合、結果は0です。


.. _sensor-jointlimitvel:

:el-prefix:`sensor/` |-| **jointlimitvel** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

この要素は速度のためのジョイントリミットセンサーを作成します。

.. _sensor-jointlimitvel-name:

.. _sensor-jointlimitvel-noise:

.. _sensor-jointlimitvel-cutoff:

.. _sensor-jointlimitvel-user:

:at:`name`, :at:`noise`, :at:`cutoff`, :at:`user`
   :ref:`CSensor` を参照してください。

.. _sensor-jointlimitvel-joint:

:at:`joint`: :at-val:`string, required`
   リミットがセンシングされるジョイント。センサー出力はmjData.efc_velからコピーされます。ジョイントリミットが違反されていない場合、結果は0です。


.. _sensor-jointlimitfrc:

:el-prefix:`sensor/` |-| **jointlimitfrc** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

この要素は制約力のためのジョイントリミットセンサーを作成します。

.. _sensor-jointlimitfrc-name:

.. _sensor-jointlimitfrc-noise:

.. _sensor-jointlimitfrc-cutoff:

.. _sensor-jointlimitfrc-user:

:at:`name`, :at:`noise`, :at:`cutoff`, :at:`user`
   :ref:`CSensor` を参照してください。

.. _sensor-jointlimitfrc-joint:

:at:`joint`: :at-val:`string, required`
   リミットがセンシングされるジョイント。センサー出力はmjData.efc_forceからコピーされます。ジョイントリミットが違反されていない場合、結果は0です。


.. _sensor-tendonlimitpos:

:el-prefix:`sensor/` |-| **tendonlimitpos** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

この要素は位置のためのテンドンリミットセンサーを作成します。

.. _sensor-tendonlimitpos-name:

.. _sensor-tendonlimitpos-noise:

.. _sensor-tendonlimitpos-cutoff:

.. _sensor-tendonlimitpos-user:

:at:`name`, :at:`noise`, :at:`cutoff`, :at:`user`
   :ref:`CSensor` を参照してください。

.. _sensor-tendonlimitpos-tendon:

:at:`tendon`: :at-val:`string, required`
   リミットがセンシングされるテンドン。センサー出力は、対応するリミット制約に対してmjData.efc_pos - mjData.efc_marginに等しくなります。テンドンリミットが違反されていない場合、結果は0です。


.. _sensor-tendonlimitvel:

:el-prefix:`sensor/` |-| **tendonlimitvel** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

この要素は速度のためのテンドンリミットセンサーを作成します。

.. _sensor-tendonlimitvel-name:

.. _sensor-tendonlimitvel-noise:

.. _sensor-tendonlimitvel-cutoff:

.. _sensor-tendonlimitvel-user:

:at:`name`, :at:`noise`, :at:`cutoff`, :at:`user`
   :ref:`CSensor` を参照してください。

.. _sensor-tendonlimitvel-tendon:

:at:`tendon`: :at-val:`string, required`
   リミットがセンシングされるテンドン。センサー出力はmjData.efc_velからコピーされます。テンドンリミットが違反されていない場合、結果は0です。


.. _sensor-tendonlimitfrc:

:el-prefix:`sensor/` |-| **tendonlimitfrc** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

この要素は制約力のためのテンドンリミットセンサーを作成します。

.. _sensor-tendonlimitfrc-name:

.. _sensor-tendonlimitfrc-noise:

.. _sensor-tendonlimitfrc-cutoff:

.. _sensor-tendonlimitfrc-user:

:at:`name`, :at:`noise`, :at:`cutoff`, :at:`user`
   :ref:`CSensor` を参照してください。

.. _sensor-tendonlimitfrc-tendon:

:at:`tendon`: :at-val:`string, required`
   リミットがセンシングされるテンドン。センサー出力はmjData.efc_forceからコピーされます。テンドンリミットが違反されていない場合、結果は0です。


.. _sensor-framepos:

:el-prefix:`sensor/` |-| **framepos** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

この要素は、オブジェクトの空間フレームの3D位置をグローバル座標で、あるいはオプションで指定された参照フレームに対して返すセンサーを作成します。

.. _sensor-framepos-name:

.. _sensor-framepos-noise:

.. _sensor-framepos-cutoff:

.. _sensor-framepos-user:

:at:`name`, :at:`noise`, :at:`cutoff`, :at:`user`
   :ref:`CSensor` を参照。

.. _sensor-framepos-objtype:

:at:`objtype`: :at-val:`[body, xbody, geom, site, camera], required`
   センサーが取り付けられるオブジェクトのタイプ。空間フレームを持つオブジェクトタイプでなければなりません。"body" はボディの慣性フレームを指し、"xbody" はボディの通常のフレーム（通常は親ボディとのジョイントを中心とする）を指します。

.. _sensor-framepos-objname:

:at:`objname`: :at-val:`string, required`
   センサーが取り付けられるオブジェクトの名前。

.. _sensor-framepos-reftype:

:at:`reftype`: :at-val:`[body, xbody, geom, site, camera]`
   参照フレームが取り付けられるオブジェクトのタイプ。意味は :at:`objtype` 属性と同じです。 :at:`reftype` と :at:`refname` が指定された場合、センサー値はこのフレームに対して測定されます。指定されない場合、センサー値はグローバルフレームに対して測定されます。

.. _sensor-framepos-refname:

:at:`refname`: :at-val:`string`
   参照フレームが取り付けられるオブジェクトの名前。


.. _sensor-framequat:

:el-prefix:`sensor/` |-| **framequat** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

この要素は、オブジェクトの空間フレームの向きを指定する単位クォータニオンをグローバル座標で返すセンサーを作成します。

.. _sensor-framequat-name:

.. _sensor-framequat-noise:

.. _sensor-framequat-cutoff:

.. _sensor-framequat-user:

:at:`name`, :at:`noise`, :at:`cutoff`, :at:`user`
   :ref:`CSensor` を参照。

.. _sensor-framequat-objtype:

:at:`objtype`: :at-val:`[body, xbody, geom, site, camera], required`
   :ref:`framepos<sensor-framepos>` センサーを参照。

.. _sensor-framequat-objname:

:at:`objname`: :at-val:`string, required`
   :ref:`framepos<sensor-framepos>` センサーを参照。

.. _sensor-framequat-reftype:

:at:`reftype`: :at-val:`[body, xbody, geom, site, camera]`
   :ref:`framepos<sensor-framepos>` センサーを参照。

.. _sensor-framequat-refname:

:at:`refname`: :at-val:`string`
   :ref:`framepos<sensor-framepos>` センサーを参照。


.. _sensor-framexaxis:

:el-prefix:`sensor/` |-| **framexaxis** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

この要素は、オブジェクトの空間フレームのX軸に対応する3D単位ベクトルをグローバル座標で返すセンサーを作成します。

.. _sensor-framexaxis-name:

.. _sensor-framexaxis-noise:

.. _sensor-framexaxis-cutoff:

.. _sensor-framexaxis-user:

:at:`name`, :at:`noise`, :at:`cutoff`, :at:`user`
   :ref:`CSensor` を参照。

.. _sensor-framexaxis-objtype:

:at:`objtype`: :at-val:`[body, xbody, geom, site, camera], required`
   :ref:`framepos<sensor-framepos>` センサーを参照。

.. _sensor-framexaxis-objname:

:at:`objname`: :at-val:`string, required`
   :ref:`framepos<sensor-framepos>` センサーを参照。

.. _sensor-framexaxis-reftype:

:at:`reftype`: :at-val:`[body, xbody, geom, site, camera]`
   :ref:`framepos<sensor-framepos>` センサーを参照。

.. _sensor-framexaxis-refname:

:at:`refname`: :at-val:`string`
   :ref:`framepos<sensor-framepos>` センサーを参照。


.. _sensor-frameyaxis:

:el-prefix:`sensor/` |-| **frameyaxis** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

この要素は、オブジェクトの空間フレームのY軸に対応する3D単位ベクトルをグローバル座標で返すセンサーを作成します。

.. _sensor-frameyaxis-name:

.. _sensor-frameyaxis-noise:

.. _sensor-frameyaxis-cutoff:

.. _sensor-frameyaxis-user:

:at:`name`, :at:`noise`, :at:`cutoff`, :at:`user`
   :ref:`CSensor` を参照。

.. _sensor-frameyaxis-objtype:

:at:`objtype`: :at-val:`[body, xbody, geom, site, camera], required`
   :ref:`framepos<sensor-framepos>` センサーを参照。

.. _sensor-frameyaxis-objname:

:at:`objname`: :at-val:`string, required`
   :ref:`framepos<sensor-framepos>` センサーを参照。

.. _sensor-frameyaxis-reftype:

:at:`reftype`: :at-val:`[body, xbody, geom, site, camera]`
   :ref:`framepos<sensor-framepos>` センサーを参照。

.. _sensor-frameyaxis-refname:

:at:`refname`: :at-val:`string`
   :ref:`framepos<sensor-framepos>` センサーを参照。


.. _sensor-framezaxis:

:el-prefix:`sensor/` |-| **framezaxis** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

この要素は、オブジェクトの空間フレームのZ軸に対応する3D単位ベクトルをグローバル座標で返すセンサーを作成します。

.. _sensor-framezaxis-name:

.. _sensor-framezaxis-noise:

.. _sensor-framezaxis-cutoff:

.. _sensor-framezaxis-user:

:at:`name`, :at:`noise`, :at:`cutoff`, :at:`user`
   :ref:`CSensor` を参照。

.. _sensor-framezaxis-objtype:

:at:`objtype`: :at-val:`[body, xbody, geom, site, camera], required`
   :ref:`framepos<sensor-framepos>` センサーを参照。

.. _sensor-framezaxis-objname:

:at:`objname`: :at-val:`string, required`
   :ref:`framepos<sensor-framepos>` センサーを参照。

.. _sensor-framezaxis-reftype:

:at:`reftype`: :at-val:`[body, xbody, geom, site, camera]`
   :ref:`framepos<sensor-framepos>` センサーを参照。

.. _sensor-framezaxis-refname:

:at:`refname`: :at-val:`string`
   :ref:`framepos<sensor-framepos>` センサーを参照。


.. _sensor-framelinvel:

:el-prefix:`sensor/` |-| **framelinvel** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

この要素は、オブジェクトの空間フレームの3D線形速度をグローバル座標で返すセンサーを作成します。

.. _sensor-framelinvel-name:

.. _sensor-framelinvel-noise:

.. _sensor-framelinvel-cutoff:

.. _sensor-framelinvel-user:

:at:`name`, :at:`noise`, :at:`cutoff`, :at:`user`
   :ref:`CSensor` を参照。

.. _sensor-framelinvel-objtype:

:at:`objtype`: :at-val:`[body, xbody, geom, site, camera], required`
   :ref:`framepos<sensor-framepos>` センサーを参照。

.. _sensor-framelinvel-objname:

:at:`objname`: :at-val:`string, required`
   :ref:`framepos<sensor-framepos>` センサーを参照。

.. _sensor-framelinvel-reftype:

:at:`reftype`: :at-val:`[body, xbody, geom, site, camera]`
   :ref:`framepos<sensor-framepos>` センサーを参照。

.. _sensor-framelinvel-refname:

:at:`refname`: :at-val:`string`
   :ref:`framepos<sensor-framepos>` センサーを参照。


.. _sensor-frameangvel:

:el-prefix:`sensor/` |-| **frameangvel** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

この要素は、オブジェクトの空間フレームの3D角速度をグローバル座標で返すセンサーを作成します。

.. _sensor-frameangvel-name:

.. _sensor-frameangvel-noise:

.. _sensor-frameangvel-cutoff:

.. _sensor-frameangvel-user:

:at:`name`, :at:`noise`, :at:`cutoff`, :at:`user`
   :ref:`CSensor` を参照。

.. _sensor-frameangvel-objtype:

:at:`objtype`: :at-val:`[body, xbody, geom, site, camera], required`
   :ref:`framepos<sensor-framepos>` センサーを参照。

.. _sensor-frameangvel-objname:

:at:`objname`: :at-val:`string, required`
   :ref:`framepos<sensor-framepos>` センサーを参照。

.. _sensor-frameangvel-reftype:

:at:`reftype`: :at-val:`[body, xbody, geom, site, camera]`
   :ref:`framepos<sensor-framepos>` センサーを参照。

.. _sensor-frameangvel-refname:

:at:`refname`: :at-val:`string`
   :ref:`framepos<sensor-framepos>` センサーを参照。


.. _sensor-framelinacc:

:el-prefix:`sensor/` |-| **framelinacc** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

この要素は、オブジェクトの空間フレームの3D線形加速度をグローバル座標で返すセンサーを作成します。

このセンサーがモデルに存在すると、センサー計算中に :ref:`mj_rnePostConstraint` の呼び出しがトリガーされます。

.. _sensor-framelinacc-name:

.. _sensor-framelinacc-noise:

.. _sensor-framelinacc-cutoff:

.. _sensor-framelinacc-user:

:at:`name`, :at:`noise`, :at:`cutoff`, :at:`user`
   :ref:`CSensor` を参照。

.. _sensor-framelinacc-objtype:

:at:`objtype`: :at-val:`[body, xbody, geom, site, camera], required`
   :ref:`framepos<sensor-framepos>` センサーを参照。

.. _sensor-framelinacc-objname:

:at:`objname`: :at-val:`string, required`
   :ref:`framepos<sensor-framepos>` センサーを参照。


.. _sensor-frameangacc:

:el-prefix:`sensor/` |-| **frameangacc** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

この要素は、オブジェクトの空間フレームの3D角加速度をグローバル座標で返すセンサーを作成します。

このセンサーがモデルに存在すると、センサー計算中に :ref:`mj_rnePostConstraint` の呼び出しがトリガーされます。

.. _sensor-frameangacc-name:

.. _sensor-frameangacc-noise:

.. _sensor-frameangacc-cutoff:

.. _sensor-frameangacc-user:

:at:`name`, :at:`noise`, :at:`cutoff`, :at:`user`
   :ref:`CSensor` を参照。

.. _sensor-frameangacc-objtype:

:at:`objtype`: :at-val:`[body, xbody, geom, site, camera], required`
   :ref:`framepos<sensor-framepos>` センサーを参照。

.. _sensor-frameangacc-objname:

:at:`objname`: :at-val:`string, required`
   :ref:`framepos<sensor-framepos>` センサーを参照。


.. _sensor-subtreecom:

:el-prefix:`sensor/` |-| **subtreecom** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

この要素は、指定されたボディを根とする運動学的サブツリーの重心をグローバル座標で返すセンサーを作成します。

.. _sensor-subtreecom-name:

.. _sensor-subtreecom-noise:

.. _sensor-subtreecom-cutoff:

.. _sensor-subtreecom-user:

:at:`name`, :at:`noise`, :at:`cutoff`, :at:`user`
   :ref:`CSensor` を参照。

.. _sensor-subtreecom-body:

:at:`body`: :at-val:`string, required`
   運動学的サブツリーが根付いているボディの名前。


.. _sensor-subtreelinvel:

:el-prefix:`sensor/` |-| **subtreelinvel** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

この要素は、指定されたボディを根とする運動学的サブツリーの重心の線形速度をグローバル座標で返すセンサーを作成します。

このセンサーがモデルに存在すると、センサー計算中に :ref:`mj_subtreeVel` の呼び出しがトリガーされます。

.. _sensor-subtreelinvel-name:

.. _sensor-subtreelinvel-noise:

.. _sensor-subtreelinvel-cutoff:

.. _sensor-subtreelinvel-user:

:at:`name`, :at:`noise`, :at:`cutoff`, :at:`user`
   :ref:`CSensor` を参照。

.. _sensor-subtreelinvel-body:

:at:`body`: :at-val:`string, required`
   運動学的サブツリーが根付いているボディの名前。


.. _sensor-subtreeangmom:

:el-prefix:`sensor/` |-| **subtreeangmom** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

この要素は、指定されたボディを根とする運動学的サブツリーの重心周りの角運動量をグローバル座標で返すセンサーを作成します。

このセンサーがモデルに存在すると、センサー計算中に :ref:`mj_subtreeVel` の呼び出しがトリガーされます。

.. _sensor-subtreeangmom-name:

.. _sensor-subtreeangmom-noise:

.. _sensor-subtreeangmom-cutoff:

.. _sensor-subtreeangmom-user:

:at:`name`, :at:`noise`, :at:`cutoff`, :at:`user`
   :ref:`CSensor` を参照。

.. _sensor-subtreeangmom-body:

:at:`body`: :at-val:`string, required`
   運動学的サブツリーが根付いているボディの名前。


.. _sensor-insidesite:

:el-prefix:`sensor/` |-| **insidesite** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
この要素は、指定されたオブジェクトがサイト内にある場合は1を、そうでない場合は0を返すセンサーを作成します。
周囲の環境ロジックでイベントをトリガーするのに便利です。
`サンプルモデル <https://github.com/google-deepmind/mujoco/blob/main/test/engine/testdata/sensor/insidesite.xml>`__ を参照してください。

.. _sensor-insidesite-name:

.. _sensor-insidesite-noise:

.. _sensor-insidesite-cutoff:

.. _sensor-insidesite-user:

:at:`name`, :at:`noise`, :at:`cutoff`, :at:`user`
   :ref:`CSensor` を参照。

.. _sensor-insidesite-objtype:

:at:`objtype`: :at-val:`[body, xbody, geom, site, camera], required`
   位置を照会されるオブジェクトのタイプ。
   :ref:`framepos<sensor-framepos>` を参照。

.. _sensor-insidesite-objname:

:at:`objname`: :at-val:`string, required`
   位置を照会されるオブジェクトの名前。
   :ref:`framepos<sensor-framepos>` を参照。

.. _sensor-insidesite-site:

:at:`site`: :at-val:`string`
   内部チェックに使用されるボリュームを定義するサイト。



.. _collision-sensors:

collision sensors
^^^^^^^^^^^^^^^^^

以下の3つのセンサータイプ、 :ref:`sensor/distance<sensor-distance>`、 :ref:`sensor/normal<sensor-normal>`、 :ref:`sensor/fromto<sensor-fromto>` は、ナローフェーズのジオム・ジオムコライダーを使用して、2つのジオムの表面間の最小符号付き距離の距離、法線方向、線分をそれぞれ測定します。衝突計算は常に実行され、標準的な衝突 :ref:`選択とフィルタリング<coSelection>` パイプラインとは独立しています。これら3つのセンサーはいくつかの共通の特性を持っています：

.. _collision-sensors-cutoff:

:at:`cutoff`
   ほとんどのセンサーでは、 :at:`cutoff` 属性は単にセンサー値のクリッピング操作を定義します。衝突センサーでは、 :ref:`mj_geomDistance` の ``dismax`` 引数に対応する、衝突が検出される最大距離を定義します。たとえば、デフォルト値の0では、 :ref:`sensor/distance<sensor-distance>` によって負の距離（ジオム・ジオムの貫入に対応）のみが報告されます。
   貫入していないジオムペアの衝突特性を判定するには、正の :at:`cutoff` が必要です。

   .. admonition:: `nativeccd` での異なる（正しい）動作
      :class: note

      :ref:`衝突検出<coDistance>` で説明されているように、 :ref:`レガシーCCDパイプライン<coCCD>` を使用すると距離が不正確になるため、その使用は推奨されません。

:at:`geom1`, :at:`geom2`, :at:`body1`, :at:`body2`
   3つすべての衝突センサータイプで、衝突する2つのジオムは :at:`geom1` と :at:`geom2` 属性を使用して明示的に指定するか、 :at:`body1`、 :at:`body2` を使用して暗黙的に指定できます。後者の場合、センサーは指定されたボディまたはボディのすべてのジオムを反復処理し（ :at:`geom1`、 :at:`body2` のような混合指定も許可されます）、最小符号付き距離を持つ衝突を選択します。

sequential sensors
   複数の衝突センサーが連続して定義され、同一の属性（ :at:`geom1`、 :at:`body1`、 :at:`geom2`、 :at:`body2`、 :at:`cutoff`）を持つ場合、たとえば同じジオムペアに対して距離と法線の両方が照会される場合、衝突関数はセンサーブロック全体に対して1回だけ呼び出され、繰り返し計算が回避されます。

.. _sensor-distance:

:el-prefix:`sensor/` |-| **distance** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

この要素は、2つのジオムの表面間の最小符号付き距離を返すセンサーを作成します。
このタイプのセンサーの詳細については、 :ref:`collision-sensors` を参照してください。

.. _sensor-distance-cutoff:

:at:`cutoff`
   この属性の意味については :ref:`collision-sensors` を参照してください。他のセンサーカテゴリとは異なります。
   衝突が検出されない場合、distanceセンサーは :at:`cutoff` 値を返すため、この場合 :at:`cutoff` は特別な意味に加えて最大クリッピング値として機能します。

.. _sensor-distance-geom1:

:at:`geom1`: :at-val:`string, optional`
   最初のジオムの名前。(:at:`geom1`、 :at:`body1`) のうち正確に1つを指定する必要があります。

.. _sensor-distance-geom2:

:at:`geom2`: :at-val:`string, optional`
   2番目のジオムの名前。(:at:`geom2`、 :at:`body2`) のうち正確に1つを指定する必要があります。

.. _sensor-distance-body1:

:at:`body1`: :at-val:`string, optional`
   最初のボディの名前。(:at:`geom1`、 :at:`body1`) のうち正確に1つを指定する必要があります。

.. _sensor-distance-body2:

:at:`body2`: :at-val:`string, optional`
   2番目のボディの名前。(:at:`geom2`、 :at:`body2`) のうち正確に1つを指定する必要があります。

.. _sensor-distance-name:

.. _sensor-distance-noise:

.. _sensor-distance-user:

:at:`name`, :at:`noise`, :at:`user`:
   :ref:`CSensor` を参照。


.. _sensor-normal:

:el-prefix:`sensor/` |-| **normal** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

この要素は、2つのジオムの表面間の最小符号付き距離の法線方向を返すセンサーを作成します。geom1の表面からgeom2の表面へ向かうことが保証されていますが、貫入の場合、この方向は一般的に重心の方向とは逆であることに注意してください。
このタイプのセンサーの詳細については、 :ref:`collision-sensors` を参照してください。

.. _sensor-normal-cutoff:

:at:`cutoff`
   この属性の意味については :ref:`collision-sensors` を参照してください。他のセンサーカテゴリとは異なります。
   衝突が検出されない場合、 :ref:`normal<sensor-normal>` センサーは (0, 0, 0) を返し、それ以外の場合は正規化された方向ベクトルを返します。このセンサーでは、 :at:`cutoff` はクランプを行いません。

.. _sensor-normal-geom1:

:at:`geom1`: :at-val:`string, optional`
   最初のジオムの名前。(:at:`geom1`、 :at:`body1`) のうち正確に1つを指定する必要があります。

.. _sensor-normal-geom2:

:at:`geom2`: :at-val:`string, optional`
   2番目のジオムの名前。(:at:`geom2`、 :at:`body2`) のうち正確に1つを指定する必要があります。

.. _sensor-normal-body1:

:at:`body1`: :at-val:`string, optional`
   最初のボディの名前。(:at:`geom1`、 :at:`body1`) のうち正確に1つを指定する必要があります。

.. _sensor-normal-body2:

:at:`body2`: :at-val:`string, optional`
   2番目のボディの名前。(:at:`geom2`、 :at:`body2`) のうち正確に1つを指定する必要があります。

.. _sensor-normal-name:

.. _sensor-normal-noise:

.. _sensor-normal-user:

:at:`name`, :at:`noise`, :at:`user`:
   :ref:`CSensor` を参照。


.. _sensor-fromto:

:el-prefix:`sensor/` |-| **fromto** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

この要素は、2つのジオムの表面間の最小符号付き距離を定義する線分を返すセンサーを作成します。線分は、ワールドフレーム内の2つの点に対応する6つの数値 (x1, y1, z1, x2, y2, z2) で定義されます。
(x1, y1, z1) はgeom1の表面上にあり、(x2, y2, z2) はgeom2の表面上にあります。このセンサーが存在し、 :ref:`mjVIS_RANGEFINDER<mjtVisFlag>` 可視化フラグが設定されている場合、線分はレンジファインダーレイとして可視化されます。
このタイプのセンサーの詳細については、 :ref:`collision-sensors` を参照してください。

.. _sensor-fromto-cutoff:

:at:`cutoff`
   この属性の意味については :ref:`collision-sensors` を参照してください。他のセンサーカテゴリとは異なります。
   衝突が検出されない場合、 :ref:`fromto<sensor-fromto>` センサーは6つの0を返します。
   このセンサーでは、 :at:`cutoff` はクランプを行いません。

.. _sensor-fromto-geom1:

:at:`geom1`: :at-val:`string, optional`
   最初のジオムの名前。(:at:`geom1`、 :at:`body1`) のうち正確に1つを指定する必要があります。

.. _sensor-fromto-geom2:

:at:`geom2`: :at-val:`string, optional`
   2番目のジオムの名前。(:at:`geom2`、 :at:`body2`) のうち正確に1つを指定する必要があります。

.. _sensor-fromto-body1:

:at:`body1`: :at-val:`string, optional`
   最初のボディの名前。(:at:`geom1`、 :at:`body1`) のうち正確に1つを指定する必要があります。

.. _sensor-fromto-body2:

:at:`body2`: :at-val:`string, optional`
   2番目のボディの名前。(:at:`geom2`、 :at:`body2`) のうち正確に1つを指定する必要があります。

.. _sensor-fromto-name:

.. _sensor-fromto-noise:

.. _sensor-fromto-user:

:at:`name`, :at:`noise`, :at:`user`:
   :ref:`CSensor` を参照。


.. _sensor-contact:

:el-prefix:`sensor/` |-| **contact** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

**動機:** 主動力学パイプライン中に発生する接触の配列は、本質的に可変サイズです。contactセンサーの目的は、接触関連情報を固定サイズの配列で報告することです。これは学習ベースのエージェントへの入力や環境ロジックに便利です。

動力学パイプラインとは独立して機能する純粋に幾何学的な :ref:`collision-sensors` とは異なり、contactセンサーは衝突と制約のステップ中に発見された情報を報告し、``mjData.{contact, efc_force}`` からデータを抽出し、 :ref:`標準<coSelection>` メカニズムによってフィルタリングされ力を生成しない接触を無視します。

contactセンサーの出力には3つの段階があります：**マッチング**、**リダクション**、**抽出**。

Matching
  :ref:`geom1<sensor-contact-geom1>`、 :ref:`geom2<sensor-contact-geom2>`、 :ref:`body1<sensor-contact-body1>`、 :ref:`body2<sensor-contact-body2>`、 :ref:`subtree1<sensor-contact-subtree1>`、 :ref:`subtree2<sensor-contact-subtree2>`、 :ref:`site<sensor-contact-site>` で定義された基準を使用して、``mjData.contact`` から接触のセットを選択します。マッチングは基準の積集合を適用します。たとえば、 :ref:`body1<sensor-contact-body1>` と :ref:`body2<sensor-contact-body2>` を設定すると、両方のボディを含む接触をマッチングし、 :ref:`geom1<sensor-contact-geom1>` のみを設定すると、そのジオムを含むすべての接触をマッチングします。 :ref:`site<sensor-contact-site>` を設定すると、サイトによって定義されるボリューム内にある接触をマッチングします。この基準は {geom2、body2、subtree2} と組み合わせて使用できます。subtree属性はボディ名を取り、ボディのサブツリー、つまりボディとそのすべての子孫を含むすべての接触をマッチングします。 :ref:`subtree1<sensor-contact-subtree1>` と :ref:`subtree2<sensor-contact-subtree2>` を同じボディに設定すると、サブツリー内の自己衝突をマッチングします。マッチング基準を指定しない場合、すべての接触をマッチングします。

Reduction
  マッチした接触の数を正確に :ref:`num<sensor-contact-num>` 個のサブ配列、つまり「スロット」に減らします。
  :at:`num` 未満の接触がマッチした場合、残りのスロットはすべて0に設定されます。デフォルトの「unsorted」リダクション基準は潜在的に非決定論的であることに注意してください。以下の :ref:`reduce<sensor-contact-reduce>` を参照してください。

Extraction
  ユーザーが指定したフィールドのセットを各スロットにコピーします。 :ref:`data<sensor-contact-data>` を参照してください。

.. _sensor-contact-geom1:
.. _sensor-contact-geom2:

:at:`geom1`, :at:`geom2`: :at-val:`string, optional`
   接触に参加するジオムの名前。上記の **matching** :ref:`sensor-contact` を参照してください。

.. _sensor-contact-body1:
.. _sensor-contact-body2:

:at:`body1`, :at:`body2`: :at-val:`string, optional`
   接触に参加するボディの名前。上記の **matching** :ref:`sensor-contact` を参照してください。

.. _sensor-contact-subtree1:
.. _sensor-contact-subtree2:

:at:`subtree1`, :at:`subtree2`: :at-val:`string, optional`
   接触に参加するサブツリーのボディの名前。上記の **matching** :ref:`sensor-contact` を参照してください。

.. _sensor-contact-site:

:at:`site`: :at-val:`string, optional`
   マッチするために接触位置がそのボリューム内になければならないサイトの名前。
   上記の **matching** :ref:`sensor-contact` を参照してください。

.. _sensor-contact-num:

:at:`num`: :at-val:`int, "1"`
   報告する接触の数。センサーは常に接触ごとに :at:`num` 個の連続データ配列（「スロット」）を報告します。
   接触が報告される順序は :ref:`reduce<sensor-contact-reduce>` 属性に依存します。

.. _sensor-contact-data:

:at:`data`: :at-val:`[found, force, torque, dist, pos, normal, tangent], "found"`
   選択された接触から報告するデータフィールドの指定。

   - :at-val:`found` **real(1)**: このフィールドには2つの目的があります。第一に、このスロットで接触が見つかったかどうかを示します。0は見つからなかったことを意味し、正の数は見つかったことを意味します。第二に、正の値は *マッチした* 接触の数と等しくなります。したがって、 :at:`num = 3` 個の接触が要求されたが2つしかマッチしなかった場合、:at-val:`found` フィールドは (2, 2, 0) になります。6つマッチした場合は (6, 6, 6) になります。
   - :at-val:`force` **real(3)**: 接触フレームでの接触力。
   - :at-val:`torque` **real(3)**: 接触フレームでの接触トルク。
   - :at-val:`dist` **real(1)**: 貫入距離。
   - :at-val:`pos`: **real(3)**: グローバルフレームでの接触位置。
   - :at-val:`normal`: **real(3)**: グローバルフレームでの接触法線方向。
   - :at-val:`tangent`: **real(3)**: グローバルフレームでの最初の接線方向。
     完全な3x3接触フレームを完成させるには、tangent2 = cross(normal, tangent) を使用します。

   重要なことに、 :at:`data` 属性には、上記でリストされた相対順序が維持されている限り、**複数の連続データタイプ** を含めることができます。たとえば、 :at:`data` = :at-val:`"found force dist"` は接触ごとに5つの数値（[found、force、dist] の連結値）を返しますが、 :at:`data` = :at-val:`"force found dist"` は :at-val:`found` が :at-val:`force` の前になければならないためエラーになります。

   Missing contacts
      :at:`num` 未満の接触がマッチング基準を満たす場合、データスロット全体が同一に0に設定されます。ほとんどのデータタイプは0を有効な値として取ることができるため、:at-val:`normal` と :at-val:`tangent` 単位ベクトルのゼロ性のみを使用して空のスロットを明確に検出できます。この理由から、:at-val:`found` データタイプは、欠落した接触の簡単な検出を可能にするために設けられています。

   Size of sensordata block
      他のセンサーとは異なり、対応するsensordataブロックのサイズは、その属性 :ref:`num<sensor-contact-num>` と :ref:`data<sensor-contact-data>` の値に依存します。contactセンサーの出力の合計サイズは ``num x size(選択されたデータフィールド)`` の積です。たとえば、 :at:`num = 6` 個の接触を :at:`data =` :at-val:`"force dist normal"` (3+1+3=7) で要求すると、42個の数値（6つの連続スロット x スロットあたり7つの数値）のsensordataブロックになります。

   Direction convention
      接触は接触するボディ間に2つの等しく反対の力を生成するため、どちらのボディがどちらに衝突するかの選択には自由度があります。

      センサーの規約は、「geom1/body1/subtree1」と「geom2/body2/subtree2」が法線の方向を決定することです。法線は常に最初から2番目を向きます。

      マッチング基準として :at:`site` のみが使用される場合、または両方のサブツリーが同じである場合など、方向を決定できない場合、法線方向は ``mjData.contact`` と同じであり、法線は最初のジオムから2番目のジオムを向き、2つのジオムは :ref:`mjtGeom` での順序に従ってソートされます。

.. _sensor-contact-reduce:

:at:`reduce`: :at-val:`[none, mindist, maxforce, netforce], "none"`
   使用するリダクション基準。上記の **reduction** :ref:`sensor-contact` も参照してください。

   - **none**: マッチング基準を満たす最初の :at:`num` 個の接触を、``mjData.contact`` に現れる順序で返します。これは最速のオプションですが、潜在的に非決定論的でもあることに注意してください。衝突検出コードへの将来の変更により、マッチする接触のアイデンティティと順序が変わる可能性があります。
   - **mindist**: 最小貫入深さを持つ :at:`num` 個の接触を昇順で返します。
   - **maxforce**: 最大力ノルムを持つ :at:`num` 個の接触を降順で返します。
   - **netforce**: このリダクション基準は、マッチしたすべての接触の力加重重心に位置する1つの新しい「合成」接触を返します。接触のフレームはグローバルフレームであるため、法線と接線方向は自然な意味を失います。力とトルクは、計算された位置で適用されたレンチがマッチしたすべての接触の組み合わせと同じ正味の効果を持つように計算されます。このリダクション基準は常に正確に1つの接触を返すことに注意してください。

.. _sensor-contact-cutoff:

:at:`cutoff`:
   この属性は無視されます。

.. _sensor-contact-name:

.. _sensor-contact-user:

.. _sensor-contact-noise:

:at:`name`, :at:`noise`, :at:`user`:
   :ref:`CSensor` を参照。

.. _sensor-tactile:

:el-prefix:`sensor/` |-| **tactile** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.. image:: images/XMLreference/tactile.png
   :align: right
   :width: 30%
   :target: https://github.com/google-deepmind/mujoco/blob/main/model/tactile/tactile.xml

触覚センサーは、センサーに関連付けられたジオムと、それに接触するSDFジオムとの間の指定された点での貫入圧力と接線フレームでの滑り速度を返します。貫入圧力を、貫入深さの関数 :math:`p(d) = \frac{d}{d_{max}-d}` として定義します。これは表面でゼロであり、最大深さに達するにつれて無限大になります。センサーはジオムとメッシュに関連付けられます。センサーは、関連付けられたジオムと他のジオムとの接触によってアクティブ化されます。ジオムフレームに配置されたメッシュの頂点は、センサー値が計算される点であるため、出力の次元はメッシュ内の頂点数の3倍です。メッシュは頂点ごとに3つの法線ベクトルを持つ必要があり、これらは接線フレームの計算に使用されます。貫入深さが正の場合（接触なし）、対応する頂点のすべての値は0です。SDF型のジオムとの接触のみがセンサー出力に寄与します。センサーは接触点の可視化を有効にすることで可視化できます。

.. _sensor-tactile-geom:

:at:`geom`: :at-val:`string, required`
   触覚センサーに関連付けるジオムの名前。

.. _sensor-tactile-mesh:

:at:`mesh`: :at-val:`string, required`
   触覚センサーに関連付けるメッシュの名前。メッシュはセンサーによって作成されます。

.. _sensor-tactile-name:

.. _sensor-tactile-user:

:at:`name`, :at:`user`:
   :ref:`CSensor` を参照。

.. _sensor-e_potential:

:el-prefix:`sensor/` |-| **e_potential** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

この要素は、ポテンシャルエネルギーを返すセンサーを作成します。

.. _sensor-e_potential-name:

.. _sensor-e_potential-noise:

.. _sensor-e_potential-cutoff:

.. _sensor-e_potential-user:

:at:`name`, :at:`noise`, :at:`cutoff`, :at:`user`
   :ref:`CSensor` を参照。


.. _sensor-e_kinetic:

:el-prefix:`sensor/` |-| **e_kinetic** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

この要素は、運動エネルギーを返すセンサーを作成します。

.. _sensor-e_kinetic-name:

.. _sensor-e_kinetic-noise:

.. _sensor-e_kinetic-cutoff:

.. _sensor-e_kinetic-user:

:at:`name`, :at:`noise`, :at:`cutoff`, :at:`user`
   :ref:`CSensor` を参照。


.. _sensor-clock:

:el-prefix:`sensor/` |-| **clock** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

この要素は、シミュレーション時間を返すセンサーを作成します。

.. _sensor-clock-name:

.. _sensor-clock-noise:

.. _sensor-clock-cutoff:

.. _sensor-clock-user:

:at:`name`, :at:`noise`, :at:`cutoff`, :at:`user`
   :ref:`CSensor` を参照。


.. _sensor-user:

:el-prefix:`sensor/` |-| **user** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

この要素は、ユーザーセンサーを作成します。MuJoCoはこのセンサーの出力の計算方法を知りません。代わりに、ユーザーはコールバック :ref:`mjcb_sensor` をインストールする必要があります。このコールバックは、``mjData.sensordata`` のセンサーデータを埋めることが期待されます。XMLでの指定は、このセンサー用のスペースを割り当て、また、どのMuJoCoオブジェクトにアタッチされるか、データを計算する前にどの計算段階が必要かを決定するために使用されます。ここで参照されるMuJoCoオブジェクトは、タプルにすることができ、タプルはMuJoCoオブジェクトのカスタムコレクション（たとえば、重心が関心のある複数のボディ）を参照できることに注意してください。

ユーザーセンサーが :ref:`stage<sensor-user-needstage>` "vel" または "acc" の場合、それぞれ :ref:`mj_subtreeVel` または :ref:`mj_rnePostConstraint` がトリガーされます。

.. _sensor-user-name:

.. _sensor-user-noise:

.. _sensor-user-cutoff:

.. _sensor-user-user:

:at:`name`, :at:`noise`, :at:`cutoff`, :at:`user`
   :ref:`CSensor` を参照。

.. _sensor-user-objtype:

:at:`objtype`: :at-val:`(any element type that can be named), optional`
   センサーがアタッチされるMuJoCoオブジェクトのタイプ。これとobjname属性が実際のオブジェクトを決定します。指定されない場合、 :ref:`mjOBJ_UNKNOWN<mjtObj>` になります。

.. _sensor-user-objname:

:at:`objname`: :at-val:`string, optional`
   センサーがアタッチされるMuJoCoオブジェクトの名前。

.. _sensor-user-datatype:

:at:`datatype`: :at-val:`[real, positive, axis, quaternion], "real"`
   このセンサーが生成する出力のタイプ。"axis" は単位長の3Dベクトルを意味します。"quat" は単位クォータニオンを意味します。
   MuJoCoがノイズを追加する際にベクトル正規化を尊重しなければならないため、これらを宣言する必要があります。"real" は、独立してノイズを追加できる実数値の一般的な配列（またはスカラー）を意味します。

.. _sensor-user-needstage:

:at:`needstage`: :at-val:`[pos, vel, acc], "acc"`
   ユーザーコールバック mjcb_sensor() がこのセンサーの出力を評価できるようになる前に完了しなければならないMuJoCo計算段階。

.. _sensor-user-dim:

:at:`dim`: :at-val:`int, required`
   このセンサーのスカラー出力の数。


.. _sensor-plugin:

:el-prefix:`sensor/` |-| **plugin** |?|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

このセンサーを :ref:`エンジンプラグイン<exPlugin>` に関連付けます。 :at:`plugin` または :at:`instance` のいずれかが必要です。

.. _sensor-plugin-plugin:

:at:`plugin`: :at-val:`string, optional`
   暗黙的なプラグインのインスタンス化に使用されるプラグイン識別子。

.. _sensor-plugin-instance:

:at:`instance`: :at-val:`string, optional`
   明示的なプラグインのインスタンス化に使用されるインスタンス名。

.. _sensor-plugin-name:

.. _sensor-plugin-cutoff:

.. _sensor-plugin-objtype:

.. _sensor-plugin-objname:

.. _sensor-plugin-reftype:

.. _sensor-plugin-refname:

.. _sensor-plugin-user:

.. |sensor/plugin attrib list| replace:: :at:`name`, :at:`cutoff`, :at:`objtype`, :at:`objname`, :at:`reftype`,
   :at:`refname`, :at:`user`

|sensor/plugin attrib list|
   :ref:`CSensor` を参照。


.. _keyframe:

**keyframe** |m|
~~~~~~~~~~~~~~~~

これはキーフレーム定義のグループ化要素です。属性はありません。キーフレームは、ユーザーにとって関心のある状態のライブラリを作成し、シミュレーション状態をライブラリ内の状態の1つに初期化するために使用できます。MuJoCoの計算には必要ありません。mjModelに割り当てられるキーフレームの数は、 :ref:`size <size>` のnkey属性と、ここで定義された要素の数のうち大きい方です。nkey個未満の要素がここで定義されている場合、未定義のキーフレームはすべてのデータが0に設定され、qpos属性のみがmjModel.qpos0に設定されます。
ユーザーは実行時にmjModelでキーフレームデータを設定することもできます。このデータは、保存されたMJCFモデルに表示されます。 :ref:`simulate.cc <saSimulate>` では、シミュレーション状態を選択されたキーフレームにコピーしたり、その逆を行ったりできることに注意してください。


.. _keyframe-key:

:el-prefix:`keyframe/` |-| **key** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

この要素は、キーフレームの1つのデータを設定します。ここに現れる順序で設定されます。指定されたベクトルで指定された要素の数が対応するmjData配列のサイズよりも少ない場合、欠落しているエントリはデフォルト設定の値に設定されます。


.. _keyframe-key-name:

:at:`name`: :at-val:`string, optional`
   このキーフレームの名前。

.. _keyframe-key-time:

:at:`time`: :at-val:`real, "0"`
   シミュレーション時間。シミュレーション状態がこのキーフレームに設定されると、mjData.timeにコピーされます。

.. _keyframe-key-qpos:

:at:`qpos`: :at-val:`real(mjModel.nq), default = mjModel.qpos0`
   ジョイント位置のベクトル。シミュレーション状態がこのキーフレームに設定されると、mjData.qposにコピーされます。

.. _keyframe-key-qvel:

:at:`qvel`: :at-val:`real(mjModel.nq), "0 0 ..."`
   ジョイント速度のベクトル。シミュレーション状態がこのキーフレームに設定されると、mjData.qvelにコピーされます。

.. _keyframe-key-act:

:at:`act`: :at-val:`real(mjModel.na), "0 0 ..."`
   アクチュエータ活性化のベクトル。シミュレーション状態がこのキーフレームに設定されると、mjData.actにコピーされます。

.. _keyframe-key-ctrl:

:at:`ctrl`: :at-val:`real(mjModel.nu), "0 0 ..."`
   制御のベクトル。シミュレーション状態がこのキーフレームに設定されると、mjData.ctrlにコピーされます。

.. _keyframe-key-mpos:

:at:`mpos`: :at-val:`real(3*mjModel.nmocap), default = mjModel.body_pos`
   モーションキャプチャボディ位置のベクトル。シミュレーション状態がこのキーフレームに設定されると、mjData.mocap_posにコピーされます。

.. _keyframe-key-mquat:

:at:`mquat`: :at-val:`real(4*mjModel.nmocap), default = mjModel.body_quat`
   モーションキャプチャボディクォータニオンのベクトル。シミュレーション状態がこのキーフレームに設定されると、mjData.mocap_quatにコピーされます。



.. _visual:

**visual** |m|
~~~~~~~~~~~~~~

この要素は、mjModelのフィールドmjModel.visに含まれる低レベル構造体mjVisualと1対1で対応しています。ここでの設定はビジュアライザー、より正確には可視化の抽象フェーズに影響を与え、後続のレンダリングのための幾何エンティティのリストを生成します。ここでの設定はグローバルであり、要素固有の可視化設定とは対照的です。グローバル設定と要素固有の設定は、重複しないプロパティを参照します。グローバル設定の一部は、要素ごとに設定できない幾何プリミティブの三角形分割などのプロパティに影響を与えます。他のグローバル設定は、装飾オブジェクト、つまり接触点や力の矢印など、モデル要素に対応しないオブジェクトのプロパティに影響を与えます。可視化設定は、意味的にいくつかのサブセクションにグループ化されています。
|br| この要素は、 :ref:`ファイルインクルード <CInclude>` メカニズムの良い候補です。「テーマ」に対応する調整された可視化設定を持つXMLファイルを作成し、そのファイルを複数のモデルにインクルードできます。

.. _visual-global:

:el-prefix:`visual/` |-| **global** |?|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

mjVisualのすべての設定はグローバルですが、ここでの設定は他のサブセクションのいずれにも適合しませんでした。したがって、これは事実上の雑多なサブセクションです。

.. _visual-global-cameraid:

:at:`cameraid`: :at-val:`int, "-1"`
   ビジュアライザーでモデルを最初にロードするときに使用されるカメラのID。デフォルト値の-1は、フリーカメラを意味します。 :ref:`モデル化されたカメラ<body-camera>` を指定するには、 :ref:`mj_name2id` で指定されるカメラのIDを使用します。

.. _visual-global-orthographic:

:at:`orthographic`: :at-val:`[false, true], "false"`
   フリーカメラが透視投影（デフォルト）を使用するか、正射投影を使用するか。この属性を設定すると、 :ref:`global/fovy<visual-global-fovy>` 属性の意味が変わります。以下を参照してください。

.. _visual-global-fovy:

:at:`fovy`: :at-val:`real, "45"`
   この属性は、フリーカメラの垂直視野を指定します。つまり、モデルにカメラが明示的に定義されていない場合でも、ビジュアライザーで常に利用可能なカメラです。カメラが透視投影を使用する場合、視野は度数で表され、グローバルな :ref:`compiler/angle <compiler-angle>` 設定に関係ありません。カメラが正射投影を使用する場合、視野は長さの単位で表されます。この場合、デフォルトの45はほとんどのシーンで大きすぎるため、おそらく減らす必要があることに注意してください。いずれの場合も、水平視野はウィンドウサイズと垂直視野から自動的に計算されます。同じ規約が :ref:`camera/fovy <body-camera-fovy>` 属性にも適用されます。

.. _visual-global-ipd:

:at:`ipd`: :at-val:`real, "0.068"`
   この属性は、フリーカメラの瞳孔間距離を指定します。ステレオスコピックモードでのレンダリングにのみ影響します。左右の視点は、対応する方向にこの値の半分だけオフセットされます。

.. _visual-global-azimuth:

:at:`azimuth`: :at-val:`real, "90"`
   この属性は、垂直z軸の周りのフリーカメラの初期方位角を度数で指定します。値0は正のx方向を見ることに対応し、デフォルト値の90は正のy方向を見ることに対応します。注視点自体は :ref:`statistic/center<statistic-center>` 属性で指定され、注視点からの距離は :ref:`statistic/extent<statistic-extent>` 属性で制御されます。

.. _visual-global-elevation:

:at:`elevation`: :at-val:`real, "-45"`
   この属性は、注視点に対するフリーカメラの初期仰角を指定します。これはカメラのX軸（ピクセル空間の右）に平行なベクトルの周りの回転であるため、*負の* 数は水平面から *上へ* カメラを移動することに対応し、その逆も同様であることに注意してください。注視点自体は :ref:`statistic/center<statistic-center>` 属性で指定され、注視点からの距離は :ref:`statistic/extent<statistic-extent>` 属性で制御されます。

.. _visual-global-linewidth:

:at:`linewidth`: :at-val:`real, "1"`
   この属性は、OpenGLの意味での線幅を指定します。ワイヤーフレームモードでのレンダリングに影響します。

.. _visual-global-glow:

:at:`glow`: :at-val:`real, "0.3"`
   この属性の値は、選択されたボディに取り付けられたすべてのジオムの放射係数に追加されます。その結果、選択されたボディが光って見えます。

.. _visual-global-realtime:

:at:`realtime`: :at-val:`real, "1"`
   この値は、`simulate` にロードされたときのモデルの初期リアルタイム係数を設定します。1: リアルタイム。1未満: リアルタイムより遅い。0より大きくなければなりません。

.. _visual-global-offwidth:

:at:`offwidth`: :at-val:`int, "640"`
   この属性と次の属性は、オフスクリーンOpenGLレンダリングバッファのピクセル単位のサイズを指定します。この属性はバッファの幅を指定します。このバッファのサイズは実行時に調整することもできますが、通常はXMLで設定する方が便利です。

.. _visual-global-offheight:

:at:`offheight`: :at-val:`int, "480"`
   この属性は、OpenGLオフスクリーンレンダリングバッファの高さをピクセル単位で指定します。

.. _visual-global-ellipsoidinertia:

:at:`ellipsoidinertia`: :at-val:`[false, true], "false"`
   この属性は、等価慣性がどのように可視化されるかを指定します。"false": ボックスを使用、"true": 楕円体を使用。

.. _visual-global-bvactive:

:at:`bvactive`: :at-val:`[false, true], "true"`
   この属性は、可視化の目的で、衝突とレイキャスティングコードがバウンディングボリューム階層の要素を交差しているとマークすべきかどうかを指定します。この属性を "false" に設定すると、高解像度メッシュを持つモデルのシミュレーションを高速化できます。

.. _visual-quality:

:el-prefix:`visual/` |-| **quality** |?|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

この要素は、レンダリングの品質に影響を与える設定を指定します。値が大きいほど品質が高くなりますが、速度が遅くなる可能性があります。 :ref:`simulate.cc <saSimulate>` は1秒あたりのフレーム数（FPS）を表示することに注意してください。目標FPSは60Hzです。ビジュアライザーに表示される数値が大幅に低い場合、これはGPUが過負荷であり、可視化を何らかの方法で簡略化する必要があることを意味します。

.. _visual-quality-shadowsize:

:at:`shadowsize`: :at-val:`int, "4096"`
   この属性は、シャドウマッピングに使用される正方形テクスチャのサイズを指定します。値が大きいほど、より滑らかな影が得られます。 :ref:`ライト <body-light>` が影を投げることができる領域のサイズも滑らかさに影響するため、これらの設定は共同で調整する必要があります。ここでのデフォルトはやや控えめです。ほとんどの最新のGPUは、速度を落とすことなく大幅に大きなテクスチャを処理できます。

.. _visual-quality-offsamples:

:at:`offsamples`: :at-val:`int, "4"`
   この属性は、オフスクリーンレンダリングのマルチサンプル数を指定します。値が大きいほど、より良いアンチエイリアシングが得られますが、GPUを遅くする可能性があります。マルチサンプリングを無効にするには、これを0に設定します。この属性はオフスクリーンレンダリングにのみ影響することに注意してください。通常のウィンドウレンダリングの場合、マルチサンプリングはウィンドウのOpenGLコンテキストが最初に作成されるときにOS依存の方法で指定され、MuJoCo内から変更することはできません。
   |br| セグメンテーション画像をレンダリングする場合、セグメンテーションインデックスを平均化しないように、マルチサンプリングは自動的に無効になります。ただし、一部のレンダリングバックエンドは自動無効化を無視します。セグメンテーション画像に不正なインデックスが含まれる場合は、この属性を手動で0に設定してみてください。

.. _visual-quality-numslices:

:at:`numslices`: :at-val:`int, "28"`
   この属性と次の3つの属性は、幾何プリミティブ用に内部生成されるメッシュの密度を指定します。このようなメッシュはレンダリングにのみ使用され、衝突検出器は基礎となる解析的表面で動作します。この値は、GLUで使用される "slices" パラメータとして、さまざまなビジュアライザー関数に渡されます。経度線に似た、Z軸周りの細分化の数を指定します。

.. _visual-quality-numstacks:

:at:`numstacks`: :at-val:`int, "16"`
   この属性の値は、GLUで使用される "stacks" パラメータとして、さまざまな可視化関数に渡されます。緯度線に似た、Z軸に沿った細分化の数を指定します。

.. _visual-quality-numquads:

:at:`numquads`: :at-val:`int, "4"`
   この属性は、ボックスの面、自動生成された平面（同じ機能を持つ要素固有の属性を持つジオム平面とは対照的）、およびハイトフィールドの側面をレンダリングするための矩形の数を指定します。幾何学的に正しいレンダリングはこの値を1に設定することで得られますが、頂点ごとの照明（フラグメントごととは対照的）を使用するため、値が大きいほど照明がよりよく機能します。


.. _visual-headlight:

:el-prefix:`visual/` |-| **headlight** |?|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

この要素は、ヘッドライトのプロパティを調整するために使用されます。モデルで明示的に定義されたライトに加えて、常に組み込みのヘッドライトがあります。ヘッドライトは、現在のカメラを中心とし、カメラが向いている方向を向いている方向性ライトです。影を投げません（いずれにせよ見えません）。ライトは加算的であるため、モデルで明示的にライトが定義されている場合、通常はヘッドライトの強度を減らす必要があることに注意してください。

.. _visual-headlight-ambient:

:at:`ambient`: :at-val:`real(3), "0.1 0.1 0.1"`
   OpenGLの意味でのヘッドライトのアンビエント成分。ここと次の2つの属性のアルファ成分は1に設定され、調整できません。

.. _visual-headlight-diffuse:

:at:`diffuse`: :at-val:`real(3), "0.4 0.4 0.4"`
   OpenGLの意味でのヘッドライトのディフューズ成分。

.. _visual-headlight-specular:

:at:`specular`: :at-val:`real(3), "0.5 0.5 0.5"`
   OpenGLの意味でのヘッドライトのスペキュラー成分。

.. _visual-headlight-active:

:at:`active`: :at-val:`int, "1"`
   この属性はヘッドライトを有効および無効にします。値0は無効を意味し、その他の値は有効を意味します。


.. _visual-map:

:el-prefix:`visual/` |-| **map** |?|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

この要素は、可視化と組み込みマウス摂動の両方に影響を与えるスケーリング量を指定するために使用されます。空間範囲に固有の次の要素のスケーリング量とは異なり、ここでの量は雑多です。

.. _visual-map-stiffness:

:at:`stiffness`: :at-val:`real, "100"`
   この属性は、マウス摂動の強度を制御します。内部摂動メカニズムは、臨界減衰、単位質量、およびここで指定された剛性を持つ質量・バネ・ダンパーをシミュレートします。値が大きいほど、選択されたボディとマウス制御のターゲット間の同じ変位に対してより大きな力が適用されます。

.. _visual-map-stiffnessrot:

:at:`stiffnessrot`: :at-val:`real, "500"`
   上記と同じですが、並進摂動ではなく回転摂動に適用されます。経験的に、回転マウス摂動が効果を持つためには、回転剛性がより大きくなければなりません。

.. _visual-map-force:

:at:`force`: :at-val:`real, "0.005"`
   この属性は、接触力と摂動力の両方の可視化を制御します。レンダリングされた力ベクトルの長さは、力の大きさにこの属性の値を掛け、モデルの平均ボディ質量（ :ref:`statistic <statistic>` 要素を参照）で割ったものに等しくなります。

.. _visual-map-torque:

:at:`torque`: :at-val:`real, "0.1"`
   上記と同じですが、力ではなく接触トルクと摂動トルクのレンダリングを制御します（現在無効）。

.. _visual-map-alpha:

:at:`alpha`: :at-val:`real, "0.3"`
   ビジュアライザーで透明性がオンになっている場合、すべての移動ボディに取り付けられたジオムがより透明になります。これは、ジオム固有のアルファ値にこの値を掛けることによって行われます。

.. _visual-map-fogstart:

:at:`fogstart`: :at-val:`real, "3"`
   ビジュアライザーは、OpenGLの意味での線形霧をシミュレートできます。霧の開始位置は、モデル範囲（ :ref:`statistic <statistic>` 要素を参照）にこの属性の値を掛けたものです。

.. _visual-map-fogend:

:at:`fogend`: :at-val:`real, "10"`
   霧の終了位置は、モデル範囲にこの属性の値を掛けたものです。

.. _visual-map-znear:

:at:`znear`: :at-val:`real, "0.01"`
   この属性と次の属性は、OpenGL投影のクリッピング平面を決定します。特にニアクリッピング平面が重要です。近すぎると、デプスバッファの解像度が（しばしば深刻に）失われ、遠すぎると、関心のあるオブジェクトがクリップされ、ズームインできなくなります。ニアクリッピング平面までの距離は、モデル ``extent`` にこの属性の値を掛けたものです。厳密に正でなければなりません。

.. _visual-map-zfar:

:at:`zfar`: :at-val:`real, "50"`
   ファークリッピング平面までの距離は、モデル ``extent`` にこの属性の値を掛けたものです。

.. _visual-map-haze:

:at:`haze`: :at-val:`real, "0.3"`
   地平線までの距離の比率で、霞で覆われます（霞のレンダリングが有効で、スカイボックスが存在する場合）。

.. _visual-map-shadowclip:

:at:`shadowclip`: :at-val:`real, "1"`
   上記のように、影の品質は、シャドウテクスチャのサイズと、特定のライトが影を投げることができる領域に依存します。方向性ライトの場合、何らかの方法で制限しない限り、領域は無限になります。この属性は、モデル範囲に現在の値を掛けた +/- として制限を指定します。これらの制限は、光の方向に直交する平面内の正方形を定義します。影がこの仮想正方形の境界を越えると、影が突然消え、正方形のエッジが現れます。

.. _visual-map-shadowscale:

:at:`shadowscale`: :at-val:`real, "0.6"`
   この属性は前のものと似た役割を果たしますが、方向性ライトではなくスポットライトに適用されます。スポットライトにはカットオフ角があり、内部的には80度に制限されています。ただし、この角度は良質な影を得るには大きすぎることが多く、影をより小さな円錐に制限する必要があります。影を投げることができる円錐の角度は、ライトのカットオフに現在の値を掛けたものです。

.. _visual-map-actuatortendon:

:at:`actuatortendon`: :at-val:`real, "2"`
   テンドンに取り付けられたアクチュエータのレンダリングのための、アクチュエータ幅のテンドン幅に対する比率。


.. _visual-scale:

:el-prefix:`visual/` |-| **scale** |?|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

この要素の設定は、さまざまな装飾オブジェクトの空間範囲を制御します。すべての場合において、レンダリングされるサイズは、平均ボディサイズ（ :ref:`statistic <statistic>` 要素を参照）に以下に文書化された属性の値を掛けたものに等しくなります。

.. _visual-scale-forcewidth:

:at:`forcewidth`: :at-val:`real, "0.1"`
   接触力と摂動力をレンダリングするために使用される矢印の半径。

.. _visual-scale-contactwidth:

:at:`contactwidth`: :at-val:`real, "0.3"`
   接触点をレンダリングするために使用される円筒の半径。円筒の法線方向は接触法線と整列されます。円筒を短く広くすると、接平面の「パンケーキ」表現になります。

.. _visual-scale-contactheight:

:at:`contactheight`: :at-val:`real, "0.1"`
   接触点をレンダリングするために使用される円筒の高さ。

.. _visual-scale-connect:

:at:`connect`: :at-val:`real, "0.2"`
   ボディとジョイントを接続するために使用されるカプセルの半径。自動生成されたスケルトンになります。

.. _visual-scale-com:

:at:`com`: :at-val:`real, "0.4"`
   運動学的サブツリーの重心をレンダリングするために使用される球の半径。

.. _visual-scale-camera:

:at:`camera`: :at-val:`real, "0.3"`
   レンダリングでモデルカメラを表すために使用される装飾オブジェクトのサイズ。

.. _visual-scale-light:

:at:`light`: :at-val:`real, "0.3"`
   レンダリングでモデルライトを表すために使用される装飾オブジェクトのサイズ。

.. _visual-scale-selectpoint:

:at:`selectpoint`: :at-val:`real, "0.2"`
   選択点（つまり、ユーザーがボディを選択するために左ダブルクリックした点）をレンダリングするために使用される球の半径。この点のローカル座標とグローバル座標は、対応するレンダリングフラグをアクティブにすることで3Dビューに印刷できることに注意してください。このようにして、関心のある点の座標を見つけることができます。

.. _visual-scale-jointlength:

:at:`jointlength`: :at-val:`real, "1.0"`
   ジョイント軸をレンダリングするために使用される矢印の長さ。

.. _visual-scale-jointwidth:

:at:`jointwidth`: :at-val:`real, "0.1"`
   ジョイント軸をレンダリングするために使用される矢印の半径。

.. _visual-scale-actuatorlength:

:at:`actuatorlength`: :at-val:`real, "0.7"`
   スカラージョイントのみに作用するアクチュエータをレンダリングするために使用される矢印の長さ。

.. _visual-scale-actuatorwidth:

:at:`actuatorwidth`: :at-val:`real, "0.2"`
   スカラージョイントのみに作用するアクチュエータをレンダリングするために使用される矢印の半径。

.. _visual-scale-framelength:

:at:`framelength`: :at-val:`real, "1.0"`
   座標フレームをレンダリングするために使用される円筒の長さ。ワールドフレームは、この設定に対して自動的にスケーリングされます。

.. _visual-scale-framewidth:

:at:`framewidth`: :at-val:`real, "0.1"`
   座標フレームをレンダリングするために使用される円筒の半径。

.. _visual-scale-constraint:

:at:`constraint`: :at-val:`real, "0.1"`
   空間制約の違反をレンダリングするために使用されるカプセルの半径。

.. _visual-scale-slidercrank:

:at:`slidercrank`: :at-val:`real, "0.2"`
   スライダー・クランクメカニズムをレンダリングするために使用されるカプセルの半径。メカニズムの2番目の部分は、この設定に対して自動的にスケーリングされます。

.. _visual-scale-frustum:

:at:`frustum`: :at-val:`real, "10"`
   錐台をレンダリングするためのカメラピンホールからzfar平面までの距離。


.. _visual-rgba:

:el-prefix:`visual/` |-| **rgba** |?|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

この要素の設定は、さまざまな装飾オブジェクトの色と透明度（rgba）を制御します。以下の用語を簡略化するために、この組み合わせた属性を「色」と呼びます。すべての値は [0 1] の範囲内にある必要があります。アルファ値0は、対応するオブジェクトのレンダリングを無効にします。

.. _visual-rgba-fog:

:at:`fog`: :at-val:`real(4), "0 0 0 1"`
   霧が有効になっている場合、すべてのピクセルの色はここで指定された色に向かってフェードします。フェードの空間範囲は、上記の :ref:`map <visual-map>` 要素のfogstart属性とfogend属性によって制御されます。

.. _visual-rgba-haze:

:at:`haze`: :at-val:`real(4), "1 1 1 1"`
   地平線での霞の色。無限平面とスカイボックスの間を滑らかに遷移させるために使用されます。デフォルトは白い霞を作成します。シームレスな遷移を作成するには、地平線近くのスカイボックスの色が平面の色/テクスチャに似ていることを確認し、霞の色をその色域のどこかに設定します。

.. _visual-rgba-force:

:at:`force`: :at-val:`real(4), "1 0.5 0.5 1"`
   摂動力をレンダリングするために使用される矢印の色。

.. _visual-rgba-inertia:

:at:`inertia`: :at-val:`real(4), "0.8 0.2 0.2 0.6"`
   等価ボディ慣性をレンダリングするために使用されるボックスの色。これは、デフォルトで透明性を持つ唯一のrgba設定です。通常、慣性ボックスの内側のジオムを見ることが望ましいためです。

.. _visual-rgba-joint:

:at:`joint`: :at-val:`real(4), "0.2 0.6 0.8 1"`
   ジョイント軸をレンダリングするために使用される矢印の色。ジョイントが制限されていて、ジョイント値が制限を超える場合、 :ref:`制約インピーダンス<soParameters>` :math:`d` の値を使用して、この色と :ref:`rgba/constraint<visual-rgba-constraint>` を混合します。

.. _visual-rgba-actuator:

:at:`actuator`: :at-val:`real(4), "0.2 0.25 0.2 1"`
   制御の中立値のアクチュエータ色。

.. _visual-rgba-actuatornegative:

:at:`actuatornegative`: :at-val:`real(4), "0.2 0.6 0.9 1"`
   制御の最も負の値のアクチュエータ色。

.. _visual-rgba-actuatorpositive:

:at:`actuatorpositive`: :at-val:`real(4), "0.9 0.4 0.2 1"`
   制御の最も正の値のアクチュエータ色。

.. _visual-rgba-com:

:at:`com`: :at-val:`real(4), "0.9 0.9 0.9 1"`
   サブツリーの重心をレンダリングするために使用される球の色。

.. _visual-rgba-camera:

:at:`camera`: :at-val:`real(4), "0.6 0.9 0.6 1"`
   レンダリングでモデルカメラを表すために使用される装飾オブジェクトの色。

.. _visual-rgba-light:

:at:`light`: :at-val:`real(4), "0.6 0.6 0.9 1"`
   レンダリングでモデルライトを表すために使用される装飾オブジェクトの色。

.. _visual-rgba-selectpoint:

:at:`selectpoint`: :at-val:`real(4), "0.9 0.9 0.1 1"`
   選択点をレンダリングするために使用される球の色。

.. _visual-rgba-connect:

:at:`connect`: :at-val:`real(4), "0.2 0.2 0.8 1"`
   ボディとジョイントを接続するために使用されるカプセルの色。自動生成されたスケルトンになります。

.. _visual-rgba-contactpoint:

:at:`contactpoint`: :at-val:`real(4), "0.9 0.6 0.2 1"`
   接触点をレンダリングするために使用される円筒の色。

.. _visual-rgba-contactforce:

:at:`contactforce`: :at-val:`real(4), "0.7 0.9 0.9 1"`
   接触力をレンダリングするために使用される矢印の色。接触力の法線成分と接線成分への分割が有効になっている場合、この色は法線成分をレンダリングするために使用されます。

.. _visual-rgba-contactfriction:

:at:`contactfriction`: :at-val:`real(4), "0.9 0.8 0.4 1"`
   分割が有効な場合にのみ、接触接線力をレンダリングするために使用される矢印の色。

.. _visual-rgba-contacttorque:

:at:`contacttorque`: :at-val:`real(4), "0.9 0.7 0.9 1"`
   接触トルクをレンダリングするために使用される矢印の色（現在無効）。

.. _visual-rgba-contactgap:

:at:`contactgap`: :at-val:`real(4), "0.5, 0.8, 0.9, 1"`
   接触ギャップに該当する接触の色（したがって接触力計算から除外されます）。

.. _visual-rgba-rangefinder:

:at:`rangefinder`: :at-val:`real(4), "1 1 0.1 1"`
   レンジファインダーセンサーをレンダリングするために使用される線ジオムの色。

.. _visual-rgba-constraint:

:at:`constraint`: :at-val:`real(4), "0.9 0 0 1"`
   空間制約違反に対応する色 -- 等式制約、ジョイント制限、テンドン制限。

.. _visual-rgba-slidercrank:

:at:`slidercrank`: :at-val:`real(4), "0.5 0.3 0.8 1"`
   スライダー・クランクメカニズムの色。

.. _visual-rgba-crankbroken:

:at:`crankbroken`: :at-val:`real(4), "0.9 0 0 1"`
   スライダー・クランクメカニズムのクランクをレンダリングするために使用される色。指定されたロッド長を維持できないモデル設定、つまり「壊れている」場合に使用されます。

.. _visual-rgba-frustum:

:at:`frustum`: :at-val:`real(4), "1 1 0 0.2"`
   カメラ錐台をレンダリングするために使用される色。

.. _visual-rgba-bv:

:at:`bv`: :at-val:`real(4), "0 1 0 0.5"`
   バウンディングボリュームをレンダリングするために使用される色。

.. _visual-rgba-bvactive:

:at:`bvactive`: :at-val:`real(4), "1 0 0 0.5"`
   :ref:`bvactive<visual-global-bvactive>` フラグが "true" の場合、アクティブなバウンディングボリュームをレンダリングするために使用される色。



.. _default:

**default** |R|
~~~~~~~~~~~~~~~

この要素は、新しいデフォルトクラスを作成するために使用されます。上記の :ref:`CDefault` を参照してください。デフォルトクラスはネストでき、すべての属性値を親から継承します。トップレベルのデフォルトクラスは常に定義されています。省略された場合は「main」と呼ばれます。

.. _default-class:

:at:`class`: :at-val:`string, required (except at the top level)`
   デフォルトクラスの名前。すべてのデフォルトクラス間で一意である必要があります。この名前は、実際のモデル要素を作成するときにクラスをアクティブにするために使用されます。


.. _default-mesh:

.. _default-mesh-scale:

.. _default-mesh-maxhullvert:

.. _default-mesh-inertia:

:el-prefix:`default/` |-| **mesh** |?|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

| この要素は、デフォルトクラスのダミー :ref:`mesh <asset-mesh>` 要素の属性を設定します。
| 使用可能な属性は :ref:`scale <asset-mesh-scale>` と :ref:`scale <asset-mesh-maxhullvert>` です。


.. _default-material:

.. _default-material-texture:

.. _default-material-emission:

.. _default-material-specular:

.. _default-material-shininess:

.. _default-material-reflectance:

.. _default-material-metallic:

.. _default-material-roughness:

.. _default-material-rgba:

.. _default-material-texrepeat:

.. _default-material-texuniform:

:el-prefix:`default/` |-| **material** |?|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

| この要素は、デフォルトクラスのダミー :ref:`material <asset-material>` 要素の属性を設定します。
| name、class を除くすべてのマテリアル属性がここで使用可能です。


.. _default-joint:

.. _default-joint-type:

.. _default-joint-group:

.. _default-joint-pos:

.. _default-joint-axis:

.. _default-joint-springdamper:

.. _default-joint-limited:

.. _default-joint-actuatorfrclimited:

.. _default-joint-actuatorgravcomp:

.. _default-joint-solreflimit:

.. _default-joint-solimplimit:

.. _default-joint-solreffriction:

.. _default-joint-solimpfriction:

.. _default-joint-stiffness:

.. _default-joint-range:

.. _default-joint-actuatorfrcrange:

.. _default-joint-margin:

.. _default-joint-ref:

.. _default-joint-springref:

.. _default-joint-armature:

.. _default-joint-damping:

.. _default-joint-frictionloss:

.. _default-joint-user:

:el-prefix:`default/` |-| **joint** |?|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

| この要素は、デフォルトクラスのダミー :ref:`joint <body-joint>` 要素の属性を設定します。
| name、class を除くすべてのジョイント属性がここで使用可能です。


.. _default-geom:

.. _default-geom-type:

.. _default-geom-pos:

.. _default-geom-quat:

.. _default-geom-contype:

.. _default-geom-conaffinity:

.. _default-geom-condim:

.. _default-geom-group:

.. _default-geom-priority:

.. _default-geom-size:

.. _default-geom-material:

.. _default-geom-friction:

.. _default-geom-mass:

.. _default-geom-density:

.. _default-geom-shellinertia:

.. _default-geom-solmix:

.. _default-geom-solref:

.. _default-geom-solimp:

.. _default-geom-margin:

.. _default-geom-gap:

.. _default-geom-fromto:

.. _default-geom-axisangle:

.. _default-geom-xyaxes:

.. _default-geom-zaxis:

.. _default-geom-euler:

.. _default-geom-hfield:

.. _default-geom-mesh:

.. _default-geom-fitscale:

.. _default-geom-rgba:

.. _default-geom-fluidshape:

.. _default-geom-fluidcoef:

.. _default-geom-user:

:el-prefix:`default/` |-| **geom** |?|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

| この要素は、デフォルトクラスのダミー :ref:`geom <body-geom>` 要素の属性を設定します。
| name、class を除くすべてのジオム属性がここで使用可能です。


.. _default-site:

.. _default-site-type:

.. _default-site-group:

.. _default-site-pos:

.. _default-site-quat:

.. _default-site-material:

.. _default-site-size:

.. _default-site-fromto:

.. _default-site-axisangle:

.. _default-site-xyaxes:

.. _default-site-zaxis:

.. _default-site-euler:

.. _default-site-rgba:

.. _default-site-user:

:el-prefix:`default/` |-| **site** |?|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

| この要素は、デフォルトクラスのダミー :ref:`site <body-site>` 要素の属性を設定します。
| name、class を除くすべてのサイト属性がここで使用可能です。

.. _default-camera-projection:

.. _default-camera-fovy:

.. _default-camera-resolution:

.. _default-camera-output:

.. _default-camera-ipd:

.. _default-camera-pos:

.. _default-camera-quat:

.. _default-camera-axisangle:

.. _default-camera-xyaxes:

.. _default-camera-zaxis:

.. _default-camera-euler:

.. _default-camera-mode:

.. _default-camera-user:

.. _default-camera-focal:

.. _default-camera-focalpixel:

.. _default-camera-principal:

.. _default-camera-principalpixel:

.. _default-camera-sensorsize:

.. _default-camera:


:el-prefix:`default/` |-| **camera** |?|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

| この要素は、デフォルトクラスのダミー :ref:`camera <body-camera>` 要素の属性を設定します。
| name、class、mode、target を除くすべてのカメラ属性がここで使用可能です。


.. _default-light:

.. _default-light-pos:

.. _default-light-dir:

.. _default-light-type:

.. _default-light-directional:

.. _default-light-castshadow:

.. _default-light-active:

.. _default-light-diffuse:

.. _default-light-intensity:

.. _default-light-ambient:

.. _default-light-specular:

.. _default-light-bulbradius:

.. _default-light-range:

.. _default-light-attenuation:

.. _default-light-cutoff:

.. _default-light-exponent:

.. _default-light-mode:

:el-prefix:`default/` |-| **light** |?|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

| この要素は、デフォルトクラスのダミー :ref:`light <body-light>` 要素の属性を設定します。
| name、class を除くすべてのライト属性がここで使用可能です。


.. _default-pair:

.. _default-pair-condim:

.. _default-pair-friction:

.. _default-pair-solref:

.. _default-pair-solreffriction:

.. _default-pair-solimp:

.. _default-pair-gap:

.. _default-pair-margin:

:el-prefix:`default/` |-| **pair** |?|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

| この要素は、デフォルトクラスのダミー :ref:`pair <contact-pair>` 要素の属性を設定します。
| name、class、geom1、geom2 を除くすべてのペア属性がここで使用可能です。


.. _default-equality:

.. _default-equality-active:

.. _default-equality-solref:

.. _default-equality-solimp:

:el-prefix:`default/` |-| **equality** |?|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

| この要素は、デフォルトクラスのダミー :ref:`equality <equality>` 要素の属性を設定します。実際の等式制約は、それらを定義するために使用されるサブ要素に応じた型を持ちます。ただし、ここではすべての等式制約型に共通の属性を設定するため、型を区別しません。
| ここで使用可能な等式制約サブ要素の属性は :at:`active`, :at:`solref`, :at:`solimp` です。


.. _default-tendon:

.. _default-tendon-group:

.. _default-tendon-limited:

.. _default-tendon-actuatorfrclimited:

.. _default-tendon-range:

.. _default-tendon-actuatorfrcrange:

.. _default-tendon-solreflimit:

.. _default-tendon-solimplimit:

.. _default-tendon-solreffriction:

.. _default-tendon-solimpfriction:

.. _default-tendon-frictionloss:

.. _default-tendon-springlength:

.. _default-tendon-width:

.. _default-tendon-material:

.. _default-tendon-margin:

.. _default-tendon-stiffness:

.. _default-tendon-damping:

.. _default-tendon-rgba:

.. _default-tendon-user:

:el-prefix:`default/` |-| **tendon** |?|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

| この要素は、デフォルトクラスのダミー :ref:`tendon <tendon>` 要素の属性を設定します。等式制約と同様に、実際のテンドンは型を持ちますが、ここではすべての型に共通の属性を設定します。
| name、class を除くすべてのテンドンサブ要素属性がここで使用可能です。


.. _default-general:

.. _default-general-ctrllimited:

.. _default-general-forcelimited:

.. _default-general-actlimited:

.. _default-general-ctrlrange:

.. _default-general-forcerange:

.. _default-general-actrange:

.. _default-general-gear:

.. _default-general-cranklength:

.. _default-general-user:

.. _default-general-group:

.. _default-general-actdim:

.. _default-general-dyntype:

.. _default-general-gaintype:

.. _default-general-biastype:

.. _default-general-dynprm:

.. _default-general-gainprm:

.. _default-general-biasprm:

.. _default-general-actearly:

:el-prefix:`default/` |-| **general** |?|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

| この要素は、デフォルトクラスのダミー :ref:`general <actuator-general>` 要素の属性を設定します。
| name、class、joint、jointinparent、site、refsite、tendon、slidersite、cranksite を除くすべてのgeneral属性がここで使用可能です。


.. _default-motor:

.. _default-motor-ctrllimited:

.. _default-motor-forcelimited:

.. _default-motor-ctrlrange:

.. _default-motor-forcerange:

.. _default-motor-gear:

.. _default-motor-cranklength:

.. _default-motor-user:

.. _default-motor-group:

:el-prefix:`default/` |-| **motor** |?|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

この要素と次の3つの要素は、 :ref:`アクチュエータショートカット <CActShortcuts>` を使用して :ref:`general <actuator-general>` 要素の属性を設定します。同じデフォルトクラス内で複数のこのようなショートカットを使用するのは意味がありません。なぜなら、それらは同じ基礎となる属性を設定し、以前の設定を置き換えるためです。 name、class、joint、jointinparent、site、refsite、tendon、slidersite、cranksite を除くすべての :ref:`motor <actuator-motor>` 属性がここで使用可能です。


.. _default-position:

.. _default-position-ctrllimited:

.. _default-position-forcelimited:

.. _default-position-ctrlrange:

.. _default-position-inheritrange:

.. _default-position-forcerange:

.. _default-position-gear:

.. _default-position-cranklength:

.. _default-position-user:

.. _default-position-group:

.. _default-position-kp:

.. _default-position-kv:

.. _default-position-dampratio:

.. _default-position-timeconst:

:el-prefix:`default/` |-| **position** |?|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

name、class、joint、jointinparent、site、refsite、tendon、slidersite、cranksite を除くすべての :ref:`position <actuator-position>` 属性がここで使用可能です。


.. _default-velocity:

.. _default-velocity-ctrllimited:

.. _default-velocity-forcelimited:

.. _default-velocity-ctrlrange:

.. _default-velocity-forcerange:

.. _default-velocity-gear:

.. _default-velocity-cranklength:

.. _default-velocity-user:

.. _default-velocity-group:

.. _default-velocity-kv:

:el-prefix:`default/` |-| **velocity** |?|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

name、class、joint、jointinparent、site、refsite、tendon、slidersite、cranksite を除くすべての :ref:`velocity <actuator-velocity>` 属性がここで使用可能です。


.. _default-intvelocity:

.. _default-intvelocity-ctrllimited:

.. _default-intvelocity-forcelimited:

.. _default-intvelocity-ctrlrange:

.. _default-intvelocity-forcerange:

.. _default-intvelocity-actrange:

.. _default-intvelocity-inheritrange:

.. _default-intvelocity-gear:

.. _default-intvelocity-cranklength:

.. _default-intvelocity-user:

.. _default-intvelocity-group:

.. _default-intvelocity-kp:

.. _default-intvelocity-kv:

.. _default-intvelocity-dampratio:

:el-prefix:`default/` |-| **intvelocity** |?|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

name、class、joint、jointinparent、site、refsite、tendon、slidersite、cranksite を除くすべての :ref:`intvelocity <actuator-intvelocity>` 属性がここで使用可能です。


.. _default-damper:

.. _default-damper-forcelimited:

.. _default-damper-ctrlrange:

.. _default-damper-forcerange:

.. _default-damper-gear:

.. _default-damper-cranklength:

.. _default-damper-user:

.. _default-damper-group:

.. _default-damper-kv:

:el-prefix:`default/` |-| **damper** |?|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

name、class、joint、jointinparent、site、refsite、tendon、slidersite、cranksite を除くすべての :ref:`damper <actuator-damper>` 属性がここで使用可能です。


.. _default-cylinder:

.. _default-cylinder-ctrllimited:

.. _default-cylinder-forcelimited:

.. _default-cylinder-ctrlrange:

.. _default-cylinder-forcerange:

.. _default-cylinder-gear:

.. _default-cylinder-cranklength:

.. _default-cylinder-user:

.. _default-cylinder-group:

.. _default-cylinder-timeconst:

.. _default-cylinder-area:

.. _default-cylinder-diameter:

.. _default-cylinder-bias:

:el-prefix:`default/` |-| **cylinder** |?|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

name、class、joint、jointinparent、site、refsite、tendon、slidersite、cranksite を除くすべての :ref:`cylinder <actuator-cylinder>` 属性がここで使用可能です。


.. _default-muscle:

.. _default-muscle-ctrllimited:

.. _default-muscle-forcelimited:

.. _default-muscle-ctrlrange:

.. _default-muscle-forcerange:

.. _default-muscle-gear:

.. _default-muscle-cranklength:

.. _default-muscle-user:

.. _default-muscle-group:

.. _default-muscle-timeconst:

.. _default-muscle-range:

.. _default-muscle-force:

.. _default-muscle-scale:

.. _default-muscle-lmin:

.. _default-muscle-lmax:

.. _default-muscle-vmax:

.. _default-muscle-fpmax:

.. _default-muscle-fvmax:

:el-prefix:`default/` |-| **muscle** |?|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

name、class、joint、jointinparent、site、refsite、tendon、slidersite、cranksite を除くすべての :ref:`muscle <actuator-muscle>` 属性がここで使用可能です。


.. _default-adhesion:

.. _default-adhesion-forcelimited:

.. _default-adhesion-ctrlrange:

.. _default-adhesion-forcerange:

.. _default-adhesion-gain:

.. _default-adhesion-user:

.. _default-adhesion-group:

:el-prefix:`default/` |-| **adhesion** |?|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

name、class、body を除くすべての :ref:`adhesion <actuator-adhesion>` 属性がここで使用可能です。


.. _custom:

**custom** |m|
~~~~~~~~~~~~~~

これは、カスタム数値要素およびテキスト要素のグループ化要素です。属性はありません。


.. _custom-numeric:

:el-prefix:`custom/` |-| **numeric** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

この要素は、mjModelにカスタム数値配列を作成します。

.. _custom-numeric-name:

:at:`name`: :at-val:`string, required`
   配列の名前。実行時に目的のカスタム要素を見つける唯一の方法はその名前を通じてであるため、この属性は必須です。

.. _custom-numeric-size:

:at:`size`: :at-val:`int, optional`
   指定された場合、この属性はデータ配列のサイズをdouble型の数で設定します。この属性が指定されていない場合、サイズは以下の実際のデータ配列から推測されます。

.. _custom-numeric-data:

:at:`data`: :at-val:`real(size), "0 0 ..."`
   mjModelにコピーされる数値データ。sizeが指定されている場合、ここで与えられる配列の長さは指定されたサイズを超えることはできません。配列の長さがより小さい場合、欠落している要素は0に設定されます。カスタム配列は実行時に情報を保存するために作成できることに注意してください。これがデータ初期化がオプションである理由です。配列サイズが省略されている場合にのみ必須になります。


.. _custom-text:

:el-prefix:`custom/` |-| **text** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

この要素は、mjModelにカスタムテキストフィールドを作成します。ユーザーコールバックやその他のカスタム計算のためにキーワードコマンドを保存するために使用できます。

.. _custom-text-name:

:at:`name`: :at-val:`string, required`
   カスタムテキストフィールドの名前。

.. _custom-text-data:

:at:`data`: :at-val:`string, required`
   mjModelにコピーされるカスタムテキスト。


.. _custom-tuple:

:el-prefix:`custom/` |-| **tuple** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

この要素は、カスタムタプルを作成します。これはMuJoCoオブジェクトのリストです。リストは、目的のオブジェクトを名前で参照することによって作成されます。

.. _custom-tuple-name:

:at:`name`: :at-val:`string, required`
   カスタムタプルの名前。


.. _tuple-element:

:el-prefix:`tuple/` |-| **element** |m|
'''''''''''''''''''''''''''''''''''''''

これにより、タプルに要素が追加されます。


.. _tuple-element-objtype:

:at:`objtype`: :at-val:`(any element type that can be named), required`
   追加されるオブジェクトのタイプ。

.. _tuple-element-objname:

:at:`objname`: :at-val:`string, required`
   追加されるオブジェクトの名前。タイプと名前は、モデルのどこかで定義された名前付きMuJoCo要素を参照する必要があります。タプルも参照できます（自己参照を含む）。

.. _tuple-element-prm:

:at:`prm`: :at-val:`real, "0"`
   タプルのこの要素に関連付けられた実数値パラメーター。その使用はユーザー次第です。


.. _extension:

**extension** |m|
~~~~~~~~~~~~~~~~~

これは、MuJoCo拡張のためのグループ化要素です。拡張により、ユーザーはカスタムコードでMuJoCoの機能を拡張できます。これについては、Programmingチャプターの :ref:`exExtension` ページで詳しく説明されています。現在、利用可能な拡張タイプは :ref:`exPlugin` のみです。

.. _extension-plugin:

:el-prefix:`extension/` |-| **plugin** |m|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

この要素は、このモデルをシミュレートするためにエンジンプラグインが必要であることを指定します。詳細については :ref:`exPlugin` を参照してください。

.. _extension-plugin-plugin:

:at:`plugin`: :at-val:`string, required`
   プラグインの識別子。

.. _plugin-instance:

:el-prefix:`plugin/` |-| **instance** |m|
'''''''''''''''''''''''''''''''''''''''''

プラグインインスタンスを宣言します。明示的なインスタンス宣言は、複数の要素が同じプラグインによってサポートされている場合、またはグローバルプラグイン設定が望ましい場合に必要です。詳細については、プラグインの :ref:`declaration<exDeclaration>` と :ref:`configuration<exConfiguration>` を参照してください。

.. _plugin-instance-name:

:at:`name`: :at-val:`string, required`
   プラグインインスタンスの名前。

.. _plugin-config:

.. _instance-config:

:el-prefix:`instance/` |-| **config** |m|
"""""""""""""""""""""""""""""""""""""""""

プラグインインスタンスの設定。モデル要素の下でプラグインを暗黙的に宣言する場合、 :el:`element/plugin/config` を使用して同一のセマンティクスで設定が実行されます。 現在プラグインをサポートしている要素は :el:`body`, :el:`composite`, :el:`actuator`, :el:`sensor` です。

.. _plugin-config-key:

.. _instance-config-key:

:at:`key`: :at-val:`string, optional`
   プラグイン設定に使用されるキー。

.. _plugin-config-value:

.. _instance-config-value:

:at:`value`: :at-val:`string, optional`
   キーに関連付けられた値。
