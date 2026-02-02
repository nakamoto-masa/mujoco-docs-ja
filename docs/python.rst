======
Python
======

MuJoCoには、C++で `pybind11 <https://pybind11.readthedocs.io/>`__ を使用して開発されたネイティブPythonバインディングが付属しています。Python APIは基盤となるC APIと一貫性があります。これにより、いくらか非Pythonic的なコード構造になります（例：関数引数の順序）が、 :doc:`APIドキュメント<APIreference/index>` が両方の言語に適用できるという利点があります。

Pythonバインディングは `PyPI <https://pypi.org/project/mujoco>`__ 上で ``mujoco`` パッケージとして配布されています。これらは、MuJoCoライブラリにできるだけ直接的にアクセスすることを目的とした低レベルのバインディングです。しかし、典型的なPythonライブラリで開発者が期待するAPIとセマンティクスを提供するため、バインディングは意図的に多くの場所で生のMuJoCo APIから逸脱しており、これらはこのページ全体で文書化されています。

Google DeepMindの `dm_control <https://github.com/google-deepmind/dm_control>`__ 強化学習ライブラリは ``mujoco`` パッケージに依存しており、Google DeepMindによって引き続きサポートされています。dm_control 1.0.0より前のバージョンに依存するコードについては、 `マイグレーションガイド <https://github.com/google-deepmind/dm_control/blob/main/migration_guide_1.0.md>`__ を参照してください。

mujoco-pyユーザー向けに、以下に :ref:`マイグレーションノート <PyMjpy_migration>` を記載しています。

.. _PyNotebook:

チュートリアルノートブック
==========================

Pythonバインディングを使用したMuJoCoチュートリアルがこちらで利用可能です： |mjcolab|

.. |mjcolab| image:: https://colab.research.google.com/assets/colab-badge.png
             :target: https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/python/tutorial.ipynb

.. _PyInstallation:

インストール
============

このパッケージのインストールには `PyPI <https://pypi.org/project/mujoco/>`__ 経由が推奨されます：

.. code-block:: shell

   pip install mujoco

MuJoCoライブラリのコピーがパッケージの一部として提供されており、個別にダウンロードやインストールを行う必要は **ありません**。

.. _PyViewer:

インタラクティブビューアー
==========================

インタラクティブGUIビューアーがPythonパッケージの一部として ``mujoco.viewer`` モジュールで提供されています。これはMuJoCoバイナリリリースに同梱される :ref:`simulate<saSimulate>` アプリケーションと同じコードベースに基づいています。3つの異なるユースケースがサポートされています： :ref:`管理ビューアー<PyViewerManaged>`、 :ref:`スタンドアロンアプリ<PyViewerApp>`、 :ref:`パッシブビューアー<PyViewerPassive>`。

.. _PyViewerManaged:

管理ビューアー
--------------

``viewer.launch`` 関数はインタラクティブビューアーを起動し、 *ユーザーコードをブロック* します。これは物理ループの正確なタイミングをサポートするのに便利です。このモードは、ユーザーコードが :ref:`エンジンプラグイン<exPlugin>` または :ref:`物理コールバック<glPhysics>` として実装され、 :ref:`mj_step` 中にMuJoCoから呼び出される場合に使用すべきです。

- ``viewer.launch()`` は空の可視化セッションを起動し、ドラッグアンドドロップでモデルをロードできます。
- ``viewer.launch(model)`` は指定された ``mjModel`` の可視化セッションを起動し、ビジュアライザーは内部的に独自の ``mjData`` インスタンスを作成します。
- ``viewer.launch(model, data)`` は上記と同じですが、ビジュアライザーは指定された ``mjData`` インスタンスで直接動作します。終了時に ``data`` オブジェクトは変更されています。

.. _PyViewerApp:

スタンドアロンアプリ
--------------------

``mujoco.viewer`` Pythonパッケージは ``if __name__ == '__main__'`` メカニズムを使用して、 :ref:`管理ビューアー<PyViewerManaged>` をコマンドラインからスタンドアロンアプリとして直接呼び出せるようにしています：

- ``python -m mujoco.viewer`` は空の可視化セッションを起動し、ドラッグアンドドロップでモデルをロードできます。
- ``python -m mujoco.viewer --mjcf=/path/to/some/mjcf.xml`` は指定されたモデルファイルの可視化セッションを起動します。

.. _PyViewerPassive:

パッシブビューアー
------------------

``viewer.launch_passive`` 関数はインタラクティブビューアーを *ブロックしない* 方法で起動し、ユーザーコードが実行を継続できるようにします。このモードでは、ユーザーのスクリプトがタイミングと物理状態の進行に責任を持ち、マウスドラッグによる摂動は、ユーザーが明示的に入力イベントを同期しない限り機能しません。

.. warning::
  MacOSでは、 ``launch_passive`` にはユーザースクリプトが特別な ``mjpython`` ランチャー経由で実行される必要があります。これは、レンダリングを行うスレッドがメインスレッドである必要があるというプラットフォームの制限を回避するために必要です。 ``mjpython`` コマンドは ``mujoco`` パッケージの一部としてインストールされ、通常の ``python`` コマンドの代替として使用でき、同一のコマンドラインフラグと引数をサポートしています。例えば、スクリプトは ``mjpython my_script.py`` で実行でき、IPythonシェルは ``mjpython -m IPython`` で起動できます。

``launch_passive`` 関数はビューアーと対話するために使用できるハンドルを返します。以下の属性があります：

- ``cam``、 ``opt``、 ``pert`` プロパティ：それぞれ :ref:`mjvCamera`、 :ref:`mjvOption`、 :ref:`mjvPerturb` 構造体に対応します。

- ``lock()``：ビューアーのミューテックスロックをコンテキストマネージャーとして提供します。ビューアーは独自のスレッドで動作するため、ユーザーコードは物理または可視化の状態を変更する前に、ビューアーロックを保持していることを確認する必要があります。これには ``launch_passive`` に渡された ``mjModel`` と ``mjData`` インスタンス、およびビューアーハンドルの ``cam``、 ``opt``、 ``pert`` プロパティも含まれます。

- ``sync(state_only=False)``：ユーザーの ``mjModel``、 ``mjData`` とGUI間で同期します。ユーザースクリプトがビューアーロックを保持せずに ``mjModel`` と ``mjData`` に任意の変更を行えるようにするため、パッシブビューアーは ``sync`` 呼び出しの外ではこれらの構造体にアクセスまたは変更しません。 ``state_only`` 引数が ``True`` の場合、すべてを同期する代わりに、 :ref:`mjSTATE_INTEGRATION<mjtState>` に対応する ``mjData`` フィールドのみが同期され、その後 :ref:`mj_forward` が呼び出されます。後者のオプションははるかに高速ですが、デフォルトケースのように任意の変更を拾うことはありません。GUI経由で行われた変更はいずれの場合も拾われますが、コード経由で例えば ``mjModel.geom_rgba`` を変更した場合、 ``state_only=False`` のときには拾われますが、 ``state_only=True`` のときには拾われません。

  ユーザースクリプトはビューアーが物理状態の変更を反映するために ``sync`` を呼び出す必要があります。 ``sync`` 関数はまた、GUIからのユーザー入力を ``mjOption`` （ ``mjModel`` 内）および ``mjData`` に転送します。これには有効/無効フラグ、制御入力、マウス摂動が含まれます。

- ``update_hfield(hfieldid)``：指定された ``hfieldid`` のハイトフィールドデータを後続のレンダリングのために更新します。

- ``update_mesh(meshid)``：指定された ``meshid`` のメッシュデータを後続のレンダリングのために更新します。

- ``update_texture(texid)``：指定された ``texid`` のテクスチャデータを後続のレンダリングのために更新します。

- ``close()``：プログラムでビューアーウィンドウを閉じます。このメソッドはロックなしで安全に呼び出せます。

- ``is_running()``：ビューアーウィンドウが実行中の場合は ``True`` を、閉じている場合は ``False`` を返します。このメソッドはロックなしで安全に呼び出せます。

- ``user_scn``：ユーザーがレンダリングフラグを変更したり、レンダリングされるシーンにカスタム可視化ジオムを追加したりできる :ref:`mjvScene` オブジェクトです。これはビューアーが最終シーンをレンダリングするために内部的に使用する ``mjvScene`` とは別で、完全にユーザーの制御下にあります。ユーザースクリプトは例えば :ref:`mjv_initGeom` や :ref:`mjv_connector` を呼び出して ``user_scn`` に可視化ジオムを追加でき、次の ``sync()`` 呼び出し時にビューアーはこれらのジオムを将来のレンダリング画像に組み込みます。同様に、ユーザースクリプトは ``user_scn.flags`` に変更を加えることができ、それは次の ``sync()`` 呼び出し時に拾われます。 ``sync()`` 呼び出しはまた、GUI経由で行われたレンダリングフラグの変更を ``user_scn`` にコピーバックして一貫性を保ちます。例：

  .. code-block:: python

    with mujoco.viewer.launch_passive(m, d, key_callback=key_callback) as viewer:

      # シーン全体のワイヤーフレームレンダリングを有効化
      viewer.user_scn.flags[mujoco.mjtRndFlag.mjRND_WIREFRAME] = 1
      viewer.sync()

      while viewer.is_running():
        ...
        # 物理をステップ
        mujoco.mj_step(m, d)

        # シーンの中央に様々な色の球の3x3x3グリッドを追加
        viewer.user_scn.ngeom = 0
        i = 0
        for x, y, z in itertools.product(*((range(-1, 2),) * 3)):
          mujoco.mjv_initGeom(
              viewer.user_scn.geoms[i],
              type=mujoco.mjtGeom.mjGEOM_SPHERE,
              size=[0.02, 0, 0],
              pos=0.1*np.array([x, y, z]),
              mat=np.eye(3).flatten(),
              rgba=0.5*np.array([x + 1, y + 1, z + 1, 2])
          )
          i += 1
        viewer.user_scn.ngeom = i
        viewer.sync()
        ...

ビューアーハンドルはコンテキストマネージャーとしても使用でき、終了時に自動的に ``close()`` を呼び出します。 ``launch_passive`` を使用するユーザースクリプトの最小例は以下のようになります。（この例は、物理が正しいウォールクロックレートで進行することを必ずしも保証しない単純な例示的な例です。）

.. code-block:: python

  import time

  import mujoco
  import mujoco.viewer

  m = mujoco.MjModel.from_xml_path('/path/to/mjcf.xml')
  d = mujoco.MjData(m)

  with mujoco.viewer.launch_passive(m, d) as viewer:
    # 30秒後にビューアーを自動的に閉じる
    start = time.time()
    while viewer.is_running() and time.time() - start < 30:
      step_start = time.time()

      # mj_stepは、物理をステップする前にポリシーを評価して
      # 制御信号を適用するコードに置き換えることができます
      mujoco.mj_step(m, d)

      # ビューアーオプションの変更例：2秒ごとに接触点を切り替え
      with viewer.lock():
        viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(d.time % 2)

      # 物理状態の変更を拾い、摂動を適用し、GUIからオプションを更新
      viewer.sync()

      # 初歩的な時間管理、ウォールクロックに対してドリフトします
      time_until_next_step = m.opt.timestep - (time.time() - step_start)
      if time_until_next_step > 0:
        time.sleep(time_until_next_step)

オプションで、 ``viewer.launch_passive`` は以下のキーワード引数を受け入れます。

- ``key_callback``：ビューアーウィンドウでキーボードイベントが発生するたびに呼び出される呼び出し可能オブジェクト。これにより、ユーザースクリプトは様々なキー押下に反応できます。例えば、スペースバーが押されたときに実行ループを一時停止または再開できます。

  .. code-block:: python

    paused = False

    def key_callback(keycode):
      if chr(keycode) == ' ':
        nonlocal paused
        paused = not paused

    ...

    with mujoco.viewer.launch_passive(m, d, key_callback=key_callback) as viewer:
      while viewer.is_running():
        ...
        if not paused:
          mujoco.mj_step(m, d)
          viewer.sync()
        ...

- ``show_left_ui`` と ``show_right_ui``：ビューアーが起動されたときにUIパネルを表示するか非表示にするかを示すブール引数。指定された値に関係なく、ユーザーは起動後もTabまたはShift+Tabを押すことでこれらのパネルの表示を切り替えることができます。

.. _PyUsage:

基本的な使い方
==============

インストール後、パッケージは ``import mujoco`` でインポートできます。構造体、関数、定数、列挙型はトップレベルの ``mujoco`` モジュールから直接利用可能です。

.. _PyStructs:

構造体
------

バインディングにはMuJoCoデータ構造を公開するPythonクラスが含まれています。最大のパフォーマンスのため、これらのクラスはコピーやバッファリングなしにMuJoCoが使用する生のメモリへのアクセスを提供します。これは、一部のMuJoCo関数（例： :ref:`mj_step`）がフィールドの内容を *インプレース* で変更することを意味します。したがって、ユーザーは必要に応じてコピーを作成することをお勧めします。例えば、ボディの位置を記録する場合、  ``positions.append(data.body('my_body').xpos.copy())`` のように書けます。  ``.copy()`` がないと、リストには同一の要素が含まれ、すべてが最新の値を指すことになります。同じことがNumPyスライスにも当てはまります。例えば、ローカル変数  ``qpos_slice = data.qpos[3:8]`` を作成してから :ref:`mj_step` を呼び出すと、  ``qpos_slice`` の値は変更されています。

`PEP 8 <https://peps.python.org/pep-0008/>`__ の命名ガイドラインに準拠するため、構造体名は大文字で始まります。例えば ``mjData`` はPythonでは ``mujoco.MjData`` になります。

``mjModel`` 以外のすべての構造体はPythonでコンストラクタを持ちます。 ``mj_defaultFoo`` スタイルの初期化関数を持つ構造体の場合、Pythonコンストラクタは自動的にデフォルト初期化子を呼び出します。例えば ``mujoco.MjOption()`` は :ref:`mj_defaultOption` で事前初期化された新しい ``mjOption`` インスタンスを作成します。それ以外の場合、Pythonコンストラクタは基礎となるC構造体をゼロ初期化します。

``mj_makeFoo`` スタイルの初期化関数を持つ構造体は、Pythonで対応するコンストラクタオーバーロードを持ちます。例えば、Pythonでの ``mujoco.MjvScene(model, maxgeom=10)`` は、Cでの ``mjv_makeScene(model, [新しいmjvSceneインスタンス], 10)`` で初期化された新しい ``mjvScene`` インスタンスを作成します。この形式の初期化が使用される場合、対応する解放関数 ``mj_freeFoo/mj_deleteFoo`` はPythonオブジェクトが削除されると自動的に呼び出されます。ユーザーがリソースを手動で解放する必要はありません。

``mujoco.MjModel`` クラスにはPythonコンストラクタがありません。代わりに、新しい :ref:`mjModel` インスタンスを作成する3つの静的ファクトリ関数を提供しています： ``mujoco.MjModel.from_xml_string``、 ``mujoco.MjModel.from_xml_path``、 ``mujoco.MjModel.from_binary_path``。最初の関数はモデルXMLを文字列として受け入れ、後の2つの関数はXMLまたはMJBモデルファイルのパスを受け入れます。3つの関数すべて、オプションでPython辞書を受け入れ、モデルコンパイル中に使用するためにMuJoCo :ref:`仮想ファイルシステム<Virtualfilesystem>` に変換されます。

.. _PyFunctions:

関数
----

MuJoCo関数は同じ名前のPython関数として公開されています。構造体とは異なり、関数名を `PEP 8 <https://peps.python.org/pep-0008/>`__ 準拠にしようとはしません。MuJoCoはアンダースコアとキャメルケースの両方を使用しているためです。ほとんどの場合、関数引数はCと全く同じように表示され、キーワード引数は :ref:`mujoco.h<inHeader>` で宣言されているのと同じ名前でサポートされています。配列入力引数を受け入れるC関数へのPythonバインディングは、NumPy配列またはNumPy配列に変換可能な反復可能オブジェクト（例：リスト）を期待します。出力引数（つまり、MuJoCoが呼び出し元に値を書き戻すことを期待する配列引数）は、常に書き込み可能なNumPy配列でなければなりません。

C APIでは、動的にサイズが決まる配列を入力として受け取る関数は、配列へのポインタ引数とともに、配列のサイズを指定する整数引数を期待します。Pythonでは、NumPy配列からサイズを自動的に（そしてより安全に）推測できるため、サイズ引数は省略されます。これらの関数を呼び出すときは、 :ref:`mujoco.h<inHeader>` に表示されるのと同じ順序で、配列サイズ以外のすべての引数を渡すか、キーワード引数を使用してください。例えば、 :ref:`mj_jac` はPythonでは ``mujoco.mj_jac(m, d, jacp, jacr, point, body)`` として呼び出す必要があります。

バインディングは、基礎となるMuJoCo関数を呼び出す前に **Pythonグローバルインタープリタロック（GIL）を解放** します。これにより、ある程度のスレッドベースの並列処理が可能になります。ただし、ユーザーはGILがMuJoCo C関数自体の実行時間のみ解放され、他のPythonコードの実行中は解放されないことに留意する必要があります。

.. note::
   バインディングが追加機能を提供する場所の1つは、トップレベルの :ref:`mj_step` 関数です。ループ内でよく呼び出されるため、基礎となる :ref:`mj_step` を何回呼び出すかを示す追加の ``nstep`` 引数を追加しました。指定されない場合、 ``nstep`` はデフォルト値の1を取ります。以下の2つのコードスニペットは同じ計算を実行しますが、最初のものは連続する物理ステップの間にGILを取得しません：

   .. code-block:: python

      mj_step(model, data, nstep=20)

   .. code-block:: python

      for _ in range(20):
        mj_step(model, data)

.. _PyEnums:

列挙型と定数
------------

MuJoCo列挙型は ``mujoco.mjtEnumType.ENUM_VALUE`` として利用可能です。例： ``mujoco.mjtObj.mjOBJ_SITE``。MuJoCo定数は、同じ名前で ``mujoco`` モジュール直下で利用可能です。例： ``mujoco.mjVISSTRING``。

.. _PyExample:

最小例
------

.. code-block:: python

   import mujoco

   XML=r"""
   <mujoco>
     <asset>
       <mesh file="gizmo.stl"/>
     </asset>
     <worldbody>
       <body>
         <freejoint/>
         <geom type="mesh" name="gizmo" mesh="gizmo"/>
       </body>
     </worldbody>
   </mujoco>
   """

   ASSETS=dict()
   with open('/path/to/gizmo.stl', 'rb') as f:
     ASSETS['gizmo.stl'] = f.read()

   model = mujoco.MjModel.from_xml_string(XML, ASSETS)
   data = mujoco.MjData(model)
   while data.time < 1:
     mujoco.mj_step(model, data)
     print(data.geom_xpos)

.. _PyNamed:

名前付きアクセス
----------------

よく設計されたMuJoCoモデルのほとんどは、関心のあるオブジェクト（ジョイント、ジオム、ボディなど）に名前を割り当てます。モデルが ``mjModel`` インスタンスにコンパイルされると、これらの名前は様々な配列メンバーのインデックスに使用される数値IDに関連付けられます。利便性とコードの可読性のため、Pythonバインディングは ``MjModel`` と ``MjData`` に「名前付きアクセス」APIを提供します。 ``mjModel`` 構造体の各 ``name_fooadr`` フィールドは名前カテゴリ ``foo`` を定義します。

各名前カテゴリ ``foo`` に対して、 ``mujoco.MjModel`` と ``mujoco.MjData`` オブジェクトは、単一の文字列引数を取り、指定された名前のエンティティ ``foo`` のすべての配列のアクセサオブジェクトを返す ``foo`` メソッドを提供します。アクセサオブジェクトには、 ``mujoco.MjModel`` または ``mujoco.MjData`` のフィールドに対応する属性が含まれますが、アンダースコアの前の部分が削除されています。さらに、アクセサオブジェクトは ``id`` と ``name`` プロパティも提供し、これらは ``mj_name2id`` と ``mj_id2name`` の代替として使用できます。例：

- ``m.geom('gizmo')`` は、  ``MjModel`` オブジェクト ``m`` 内の"gizmo"という名前のジオムに関連する配列のアクセサを返します。
- ``m.geom('gizmo').rgba`` は長さ4のNumPy配列ビューで、ジオムのRGBA色を指定します。具体的には、  ``m.geom_rgba[4*i:4*i+4]`` の部分に対応します。ここで  ``i = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, 'gizmo')`` です。
- ``m.geom('gizmo').id`` は  ``mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, 'gizmo')`` が返すのと同じ数値です。
- ``m.geom(i).name`` は  ``'gizmo'`` です。ここで  ``i = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, 'gizmo')`` です。

さらに、Python APIは、MJCFスキーマでそのカテゴリのエンティティを定義するXML要素名に対応する、いくつかの名前カテゴリのエイリアスを定義しています。例えば、 ``m.joint('foo')`` は ``m.jnt('foo')`` と同じです。これらのエイリアスの完全なリストは以下に示します。

ジョイント用のアクセサは他のカテゴリとは少し異なります。一部の ``mjModel`` と ``mjData`` フィールド（サイズ ``nq`` または ``nv`` のもの）は、ジョイントではなく自由度（DOF）に関連付けられています。これは、異なるタイプのジョイントが異なる数のDOFを持つためです。それでも、これらのフィールドを対応するジョイントに関連付けます。例えば ``d.joint('foo').qpos`` や ``d.joint('foo').qvel`` などです。ただし、これらの配列のサイズは、ジョイントのタイプに応じてアクセサ間で異なります。

名前付きアクセスは、モデル内のエンティティ数に対してO(1)であることが保証されています。つまり、名前でエンティティにアクセスする時間は、モデル内の名前やエンティティの数に応じて増加しません。

完全性のため、ここではMuJoCoのすべての名前カテゴリの完全なリストと、Python APIで定義されている対応するエイリアスを示します。

- ``body``
- ``jnt`` または ``joint``
- ``geom``
- ``site``
- ``cam`` または ``camera``
- ``light``
- ``mesh``
- ``skin``
- ``hfield``
- ``tex`` または ``texture``
- ``mat`` または ``material``
- ``pair``
- ``exclude``
- ``eq`` または ``equality``
- ``tendon`` または ``ten``
- ``actuator``
- ``sensor``
- ``numeric``
- ``text``
- ``tuple``
- ``key`` または ``keyframe``

.. _PyRender:

レンダリング
------------

MuJoCo自体は、 ``mjr_`` レンダリングルーチンを呼び出す前に、ユーザーが動作するOpenGLコンテキストをセットアップすることを期待しています。Pythonバインディングは、ユーザーがオフスクリーンレンダリング用のコンテキストをセットアップするのに役立つ基本クラス ``mujoco.GLContext`` を提供しています。コンテキストを作成するには、 ``ctx = mujoco.GLContext(max_width, max_height)`` を呼び出します。コンテキストが作成されたら、MuJoCoレンダリング関数を呼び出す前に、 ``ctx.make_current()`` でカレントにする必要があります。コンテキストは、任意の時点で1つのスレッドでのみカレントにでき、すべての後続のレンダリング呼び出しは同じスレッドで行う必要があります。

コンテキストは ``ctx`` オブジェクトが削除されると自動的に解放されますが、一部のマルチスレッドシナリオでは、基礎となるOpenGLコンテキストを明示的に解放する必要がある場合があります。そのためには、 ``ctx.free()`` を呼び出します。その後、そのコンテキストでそれ以上レンダリング呼び出しが行われないことを確認するのはユーザーの責任です。

コンテキストが作成されたら、ユーザーは :ref:`Visualization` セクションに記載されているように、MuJoCoの標準レンダリングに従うことができます。

.. _PyError:

エラー処理
----------

MuJoCoは :ref:`mju_error` メカニズムを介して回復不可能なエラーを報告し、プロセス全体を即座に終了します。ユーザーは :ref:`mju_user_error` コールバックを介してカスタムエラーハンドラをインストールできますが、これもプロセスを終了することが期待されており、それ以外の場合、コールバックが返った後のMuJoCoの動作は未定義です。実際には、エラーコールバックが *MuJoCoに* 返らなければ十分であり、 `longjmp <https://en.cppreference.com/w/c/program/longjmp>`__ を使用してMuJoCoの呼び出しスタックを外部の呼び出し元にスキップすることは許可されています。

Pythonバインディングはlongjmpを利用して、回復不可能なMuJoCoエラーを、通常のPython的な方法でキャッチおよび処理できる ``mujoco.FatalError`` タイプのPython例外に変換します。さらに、現在プライベートなAPIを使用して、スレッドローカルな方法でエラーコールバックをインストールするため、複数のスレッドからMuJoCoへの並行呼び出しが可能になります。

.. _PyCallbacks:

コールバック
------------

MuJoCoは、ユーザーが計算パイプラインの特定部分を変更するためにカスタムコールバック関数をインストールできるようにしています。例えば、 :ref:`mjcb_sensor` はカスタムセンサーの実装に使用でき、 :ref:`mjcb_control` はカスタムアクチュエータの実装に使用できます。コールバックは :ref:`mujoco.h<inHeader>` で ``mjcb_`` プレフィックスが付いた関数ポインタを介して公開されています。

各コールバック ``mjcb_foo`` に対して、ユーザーは ``mujoco.set_mjcb_foo(some_callable)`` でPython呼び出し可能オブジェクトに設定できます。リセットするには、 ``mujoco.set_mjcb_foo(None)`` を呼び出します。現在インストールされているコールバックを取得するには、 ``mujoco.get_mjcb_foo()`` を呼び出します。（ゲッターは、コールバックがPythonバインディングを介してインストールされていない場合は使用 **すべきではありません**。）バインディングは、コールバックに入るたびに自動的にGILを取得し、MuJoCoに再入する前に解放します。コールバックはMuJoCoの計算パイプライン全体で数回トリガーされるため、これはおそらく深刻なパフォーマンスへの影響を及ぼし、「本番」ユースケースには適していないでしょう。ただし、この機能は複雑なモデルのプロトタイピングに役立つことが期待されます。

あるいは、コールバックがネイティブ動的ライブラリに実装されている場合、ユーザーは `ctypes <https://docs.python.org/3/library/ctypes.html>`__ を使用してC関数ポインタへのPythonハンドルを取得し、それを ``mujoco.set_mjcb_foo`` に渡すことができます。バインディングはその後、基礎となる関数ポインタを取得し、生のコールバックポインタに直接割り当て、コールバックに入るたびにGILは **取得されません**。

.. _PyModelEdit:

モデル編集
==========
モデル編集のためのC APIは :doc:`Programming<../programming/modeledit>` の章で文書化されています。この機能はPython APIに反映されており、いくつかの便利なメソッドが追加されています。以下は最小限の使用例です。より多くの例はModel Editing `colabノートブック <https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/python/mjspec.ipynb>`__ で見つけることができます。


.. code-block:: python

   import mujoco
   spec = mujoco.MjSpec()
   spec.modelname = "my model"
   body = spec.worldbody.add_body(
       pos=[1, 2, 3],
       quat=[0, 1, 0, 0],
   )
   geom = body.add_geom(
       name='my_geom',
       type=mujoco.mjtGeom.mjGEOM_SPHERE,
       size=[1, 0, 0],
       rgba=[1, 0, 0, 1],
   )
   ...
   model = spec.compile()

構築
----

``MjSpec`` オブジェクトは :ref:`mjSpec` 構造体をラップし、3つの方法で構築できます：

1. 空のspecを作成： ``spec = mujoco.MjSpec()``
2. XML文字列からspecをロード： ``spec = mujoco.MjSpec.from_string(xml_string)``
3. XMLファイルからspecをロード： ``spec = mujoco.MjSpec.from_file(file_path)``

``from_string()`` と ``from_file()`` メソッドは構築時にのみ呼び出せることに注意してください。

アセット
^^^^^^^^

3つのメソッドすべて、XML内のアセット参照を解決するために使用される ``assets`` というオプション引数を取ります。この引数は、アセット名（文字列）をアセットデータ（バイト）にマッピングする辞書です。以下に示します：

.. code-block:: python

  assets = {'image.png': b'image_data'}
  spec = mujoco.MjSpec.from_string(xml_referencing_image_png, assets=assets)
  model = spec.compile()

XMLへの保存
-----------

コンパイルされた ``MjSpec`` オブジェクトは ``to_xml()`` メソッドでXML文字列に保存できます：

.. code-block:: python

   print(spec.to_xml())

.. code-block:: XML

   <mujoco model="my model">
     <compiler angle="radian"/>

     <worldbody>
       <body pos="1 2 3" quat="0 1 0 0">
         <geom name="my_geom" size="1" rgba="1 0 0 1"/>
       </body>
     </worldbody>
   </mujoco>

アタッチメント
--------------

複数のspecをアタッチメントを使用して結合することが可能です。以下のオプションが利用可能です：

-   親specのフレームに子specのボディをアタッチ： ``body.attach_body(body, prefix, suffix)``、アタッチされたボディへの参照を返します。入力として使用されたボディと同一であるべきです。
-   親specのボディに子specのフレームをアタッチ： ``body.attach_frame(frame, prefix, suffix)``、アタッチされたフレームへの参照を返します。入力として使用されたフレームと同一であるべきです。
-   親specのサイトに子specをアタッチ： ``parent_spec.attach(child_spec, site=site_name_or_obj)``、フレームへの参照を返します。これはアタッチされたworldbodyがフレームに変換されたものです。サイトは子specに属している必要があります。プレフィックスとサフィックスもキーワード引数として指定できます。
-   親specのフレームに子specをアタッチ： ``parent_spec.attach(child_spec, frame=frame_name_or_obj)``、フレームへの参照を返します。これはアタッチされたworldbodyがフレームに変換されたものです。フレームは子specに属している必要があります。プレフィックスとサフィックスもキーワード引数として指定できます。

アタッチのデフォルト動作はコピーしないことです。したがって、すべての子参照（worldbody以外）は親で引き続き有効であり、子を変更すると親が変更されます。これは、MJCFの :ref:`attach<body-attach>` および :ref:`replicate<replicate>` メタ要素の動作とは異なります。これらはアタッチ中にディープコピーを作成します。ただし、 ``spec.copy_during_attach`` を ``True`` に設定することで、デフォルトの動作をオーバーライドできます。この場合、子specがコピーされ、子への参照は親を指さなくなります。

.. code-block:: python

   import mujoco

   # 親specを作成
   parent = mujoco.MjSpec()
   body = parent.worldbody.add_body()
   frame = parent.worldbody.add_frame()
   site = parent.worldbody.add_site()

   # 子specを作成
   child = mujoco.MjSpec()
   child_body = child.worldbody.add_body()
   child_frame = child.worldbody.add_frame()

   # 異なる方法で子を親にアタッチ
   body_in_frame = frame.attach_body(child_body, 'child-', '')
   frame_in_body = body.attach_frame(child_frame, 'child-', '')
   worldframe_in_site = parent.attach(child, site=site, prefix='child-')
   worldframe_in_frame = parent.attach(child, frame=frame, prefix='child-')

.. _PyEditConvenience:

便利なメソッド
--------------

Pythonバインディングは、モデル編集を容易にするために、C APIで直接利用できない多数の便利なメソッドと属性を提供しています：

名前付きアクセス
^^^^^^^^^^^^^^^^
``MjSpec`` オブジェクトには、要素の名前付きアクセスのための ``.body()、 .joint()、 .site()、 ...`` のようなメソッドがあります。 ``spec.geom('my_geom')`` は"my_geom"という名前の :ref:`mjsGeom` を返すか、存在しない場合は ``None`` を返します。

要素リスト
^^^^^^^^^^
spec内のすべての要素のリストは、複数形を使用した名前付きプロパティでアクセスできます。例えば、 ``spec.meshes`` はspec内のすべてのメッシュのリストを返します。以下のプロパティが実装されています： ``sites``、 ``geoms``、 ``joints``、 ``lights``、 ``cameras``、 ``bodies``、 ``frames``、 ``materials``、 ``meshes``、 ``pairs``、 ``equalities``、 ``tendons``、 ``actuators``、 ``skins``、 ``textures``、 ``texts``、 ``tuples``、 ``flexes``、 ``hfields``、 ``keys``、 ``numerics``、 ``excludes``、 ``sensors``、 ``plugins``。

要素の削除
^^^^^^^^^^
``delete()`` メソッドは、対応する要素をspecから削除します。例えば ``spec.delete(spec.geom('my_geom'))`` は"my_geom"という名前のジオムと、それを参照するすべての要素を削除します。子を持つことができる要素（ボディとデフォルト）の場合、 ``delete`` はすべての子も削除します。ボディサブツリーを削除するとき、サブツリー内の要素を参照するすべての要素も削除されます。

ツリーのトラバーサル
^^^^^^^^^^^^^^^^^^^^
キネマティックツリーのトラバーサルは、ツリー関連の要素リストを返す以下のメソッドによってサポートされています：

直接の子：
  上記のspec レベルの要素リストと同様に、ボディにはすべての直接の子を返すプロパティがあります。例えば、 ``body.geoms`` はボディの直接の子であるすべてのジオムのリストを返します。これは、ツリー内のすべての要素、すなわち ``bodies``、 ``joints``、 ``geoms``、 ``sites``、 ``cameras``、 ``lights``、 ``frames`` で機能します。

再帰的検索：
  ``body.find_all()`` は、指定されたボディのサブツリー内にある、指定されたタイプのすべての要素のリストを返します。要素タイプは :ref:`mjtObj` 列挙型、または対応する文字列で指定できます。例えば、 ``body.find_all(mujoco.mjtObj.mjOBJ_SITE)`` または ``body.find_all('site')`` のいずれかで、ボディ配下のすべてのサイトのリストが返されます。

親：
  指定された要素の親ボディは ``parent`` プロパティでアクセスできます。これにはボディとフレームが含まれます。例えば、サイトの親は ``site.parent`` でアクセスできます。

シリアライゼーション
^^^^^^^^^^^^^^^^^^^^
``MjSpec`` オブジェクトは、すべてのアセットとともに ``spec.to_zip(file)`` 関数を使用してシリアライズできます。ここで ``file`` はファイルへのパスまたはファイルオブジェクトです。zipファイルからspecをロードするには、 ``spec = MjSpec.from_zip(file)`` を使用します。ここで ``file`` はzipファイルへのパスまたはzipファイルオブジェクトです。

メッシュ作成
^^^^^^^^^^^^
:ref:`mjsMesh` オブジェクトには、 :ref:`mesh/builtin<asset-mesh-builtin>` セマンティクスに対応する名前付き属性でモデルを作成するための便利なメソッドが含まれています。 `specs_test.py <https://github.com/google-deepmind/mujoco/blob/main/python/mujoco/specs_test.py>`__ を参照してください。

.. code-block:: python

   mesh = spec.add_mesh(name='prism')
   mesh.make_cone(nedge=5, radius=1)

テクスチャ編集
^^^^^^^^^^^^^^
:ref:`mjsTexture` バッファオプションは、 ``data`` 属性にテクスチャバイトを格納します。この属性は読み取りおよび変更できます。例：

.. code-block:: python

  texture = spec.add_texture(name='texture', height=1, width=3, nchannel=3)
  texture.data = bytes([255, 0, 0, 0, 255, 0, 0, 0, 255])  # 赤、緑、青のピクセルを割り当て
  texture.data[1] = 255  # 最初のピクセルを黄色に変更

.. _PyMJCF:

``PyMJCF`` と ``bind`` との関係
-------------------------------

`dm_control <https://github.com/google-deepmind/dm_control/tree/main>`__ の `PyMJCF <https://github.com/google-deepmind/dm_control/blob/main/dm_control/mjcf/README.md>`__ モジュールは、ここで説明したネイティブモデル編集APIと同様の機能を提供しますが、文字列のPython操作に依存しているため、約2桁遅くなります。

``PyMJCF`` に慣れているユーザーのために、 ``MjSpec`` オブジェクトは概念的に ``dm_control`` の ``mjcf_model`` に似ています。より詳細なマイグレーションガイドは将来ここに追加される可能性があります。その間、Model Editing `colabノートブック <https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/python/mjspec.ipynb>`__ には、 ``dm_control`` `チュートリアルノートブック <https://github.com/google-deepmind/dm_control/blob/main/dm_control/mjcf/tutorial.ipynb>`__ の ``PyMJCF`` 例の再実装が含まれています。

``PyMJCF`` は「バインディング」の概念を提供し、ヘルパークラスを介して :ref:`mjModel` と :ref:`mjData` の値へのアクセスを提供します。ネイティブAPIでは、ヘルパークラスは必要ないため、 ``mjs`` オブジェクトを :ref:`mjModel` と :ref:`mjData` に直接バインドすることができます。例えば、名前に"torso"という文字列を含む複数のジオムがあるとします。 ``mjData`` からXY平面でのデカルト座標位置を取得したいとします。これは次のように行えます：

.. code-block:: python

   torsos = [data.bind(geom) for geom in spec.geoms if 'torso' in geom.name]
   pos_x = [torso.xpos[0] for torso in torsos]
   pos_y = [torso.xpos[1] for torso in torsos]

``bind`` メソッドを使用するには、 :ref:`mjModel` と :ref:`mjData` が :ref:`mjSpec` からコンパイルされている必要があります。最後のコンパイル以降に :ref:`mjSpec` からオブジェクトが追加または削除された場合、エラーが発生します。

ノート
------

- :ref:`mj_recompile` はC APIとは動作が異なります。C APIではモデルとデータをインプレースで変更しますが、Python APIでは新しい :ref:`MjModel` と :ref:`MjData` オブジェクトを返します。これは、ダングリング参照を避けるためです。

.. _PyBuild:

ソースからのビルド
==================

.. note::
    ソースからのビルドは、Pythonバインディングを変更している場合（または非常に古いLinuxシステムで実行しようとしている場合）にのみ必要です。そうでない場合は、PyPIからのビルド済みバイナリのインストールをお勧めします。

1. CMakeとC++17コンパイラがインストールされていることを確認してください。

2. GitHubから ``mujoco`` リポジトリ全体をクローンします。

   .. code-block:: shell

     git clone https://github.com/google-deepmind/mujoco.git

3. MuJoCoをインストールします。GitHubから `最新のバイナリリリース <https://github.com/google-deepmind/mujoco/releases>`__ をダウンロードするか（macOSでは、ダウンロードはDMGファイルに対応し、ダブルクリックまたは ``hdiutil attach <dmg_file>`` を実行してマウントできます）、 :ref:`inBuild` の指示に従ってソースから *ビルド* および *インストール* します。

4. クローンしたMuJoCoコードベースのpythonディレクトリに ``cd`` します：

   .. code-block:: shell

      cd mujoco/python

5. 仮想環境を作成します：

   .. code-block:: shell

      python3 -m venv /tmp/mujoco
      source /tmp/mujoco/bin/activate

6. ``make_sdist.sh`` スクリプトで `ソースディストリビューション <https://packaging.python.org/en/latest/glossary/#term-Source-Distribution-or-sdist>`__ tarballを生成します。

   .. code-block:: shell

      bash make_sdist.sh

   ``make_sdist.sh`` スクリプトは、バインディングのビルドに必要な追加のC++ヘッダファイルを生成し、 ``python`` ディレクトリ外のリポジトリの他の場所から必要なファイルをsdistに取り込みます。完了すると、スクリプトは ``mujoco-x.y.z.tar.gz`` ファイル（ ``x.y.z`` はバージョン番号）を含む ``dist`` ディレクトリを作成します。

7. 生成されたソースディストリビューションを使用して、バインディングをビルドおよびインストールします。先ほどダウンロードまたはビルドしてインストールしたMuJoCoライブラリへのパスを ``MUJOCO_PATH`` 環境変数で指定し、MuJoCoプラグインディレクトリへのパスを ``MUJOCO_PLUGIN_PATH`` 環境変数で指定する必要があります。 ``MUJOCO_PLUGIN_PATH`` 環境変数は、クローンしたMuJoCoコードベースの ``plugin`` フォルダを指すことができます。

   .. note::
      macOSの場合、ファイルはDMGから抽出する必要があります。ステップ2のようにマウントすると、 ``mujoco.framework`` ディレクトリは ``/Volumes/MuJoCo`` にあり、pluginsディレクトリは ``/Volumes/MuJoCo/MuJoCo.app/Contents/MacOS/mujoco_plugin`` にあります。これら2つのディレクトリを便利な場所にコピーするか、 ``MUJOCO_PATH=/Volumes/MuJoCo MUJOCO_PLUGIN_PATH=/Volumes/MuJoCo/MuJoCo.app/Contents/MacOS/mujoco_plugin`` を使用できます。

   .. code-block:: shell

      cd dist
      MUJOCO_PATH=/PATH/TO/MUJOCO \
      MUJOCO_PLUGIN_PATH=/PATH/TO/MUJOCO/PLUGIN \
      pip install mujoco-x.y.z.tar.gz

Pythonバインディングがインストールされました！正しくインストールされたことを確認するには、 ``mujoco`` ディレクトリの外に ``cd`` して、 ``python -c "import mujoco"`` を実行してください。

.. tip::
   参考として、動作するビルド構成は、GitHubのMuJoCoの `継続的インテグレーションセットアップ <https://github.com/google-deepmind/mujoco/blob/main/.github/workflows/build.yml>`_ にあります。


.. _PyModule:

モジュール
==========

``mujoco`` パッケージには2つのサブモジュールが含まれています： ``mujoco.rollout`` と ``mujoco.minimize``

.. _PyRollout:

rollout
-------
``mujoco.rollout`` と ``mujoco.rollout.Rollout`` は、pybind11を介してPythonモジュールとして公開される追加のC/C++機能を追加する方法を示しています。これは `rollout.cc <https://github.com/google-deepmind/mujoco/blob/main/python/mujoco/rollout.cc>`__ に実装され、 `rollout.py <https://github.com/google-deepmind/mujoco/blob/main/python/mujoco/rollout.py>`__ でラップされています。このモジュールは、Python外で実装された緊密なループが有益な一般的なユースケースに対応しています：初期状態と制御のシーケンスを与えて軌跡をロールアウトし（つまり、ループ内で :ref:`mj_step` を呼び出し）、後続の状態とセンサー値を返します。複数のMjDataインスタンス（スレッドごとに1つ）が引数として渡された場合、ロールアウトは内部的に管理されたスレッドプールで並列に実行されます。このノートブックは、 ``rollout`` |rollout_colab| の使用方法と、いくつかのベンチマーク（例：以下の図）を示しています。

.. |rollout_colab| image:: https://colab.research.google.com/assets/colab-badge.png
                   :target: https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/python/rollout.ipynb

.. image:: images/python/rollout.png
   :align: right
   :width: 97%

基本的な使用形式は次のとおりです：

.. code-block:: python

   state, sensordata = rollout.rollout(model, data, initial_state, control)

- ``model`` は単一のMjModelインスタンス、または長さ ``nbatch`` の同質なMjModelのシーケンスです。同質なモデルは同じ整数サイズを持ちますが、浮動小数点値は異なる場合があります。
- ``data`` は単一のMjDataインスタンス、または長さ ``nthread`` の互換性のあるMjDataのシーケンスです。
- ``initial_state`` は ``nbatch x nstate`` 配列で、サイズ ``nstate`` の ``nbatch`` 個の初期状態を持ちます。ここで ``nstate = mj_stateSize(model, mjtState.mjSTATE_FULLPHYSICS)`` は :ref:`完全な物理状態<siFullPhysics>` のサイズです。
- ``control`` は ``nbatch x nstep x ncontrol`` の制御配列です。デフォルトでは、制御は ``mjModel.nu`` 個の標準アクチュエータですが、オプションの ``control_spec`` ビットフラグを渡すことで、 :ref:`ユーザー入力<siInput>` 配列の任意の組み合わせを指定できます。

ロールアウトが発散した場合、現在の状態とセンサー値を使用して軌跡の残りを埋めます。したがって、時間値が増加しないことを使用して、発散したロールアウトを検出できます。

``rollout`` 関数は計算的にステートレスになるように設計されているため、ステッピングパイプラインのすべての入力が設定され、与えられた ``MjData`` インスタンスに既に存在する値は出力に影響しません。

デフォルトでは、 ``rollout.rollout`` は ``len(data) > 1`` の場合、呼び出しごとに新しいスレッドプールを作成します。複数の呼び出しにわたってスレッドプールを再利用するには、 ``persistent_pool`` 引数を使用します。 ``rollout.rollout`` は、永続プールを使用する場合、スレッドセーフではありません。基本的な使用形式は次のとおりです：

.. code-block:: python

   state, sensordata = rollout.rollout(model, data, initial_state, persistent_pool=True)

プールはインタープリタのシャットダウン時、または ``rollout.shutdown_persistent_pool`` の呼び出しによってシャットダウンされます。

複数のスレッドから複数のスレッドプールを使用するには、 ``Rollout`` オブジェクトを使用します。基本的な使用形式は次のとおりです：

.. code-block:: python

   # ブロックを抜けるときにプールをシャットダウン
   with rollout.Rollout(nthread=nthread) as rollout_:
    rollout_.rollout(model, data, initial_state)

または

.. code-block:: python

   # オブジェクトの削除時、またはrollout_.close()の呼び出し時にプールをシャットダウン
   # スレッドのクリーンなシャットダウンを確保するには、インタープリタ終了前にclose()を呼び出します
   rollout_ = rollout.Rollout(nthread=nthread)
   rollout_.rollout(model, data, initial_state)
   rollout_.close()

グローバルインタープリタロックが解放されるため、この関数はPythonスレッドを使用してスレッド化することもできます。ただし、これはネイティブスレッドを使用するよりも効率が低くなります。スレッド化された操作の例（および一般的な使用例）については、 `rollout_test.py <https://github.com/google-deepmind/mujoco/blob/main/python/mujoco/rollout_test.py>`__ の ``test_threading`` 関数を参照してください。

.. _PyMinimize:

minimize
--------

このモジュールには最適化関連のユーティリティが含まれています。

``minimize.least_squares()`` 関数は、 :ref:`mju_boxQP` で逐次2次計画問題を解く非線形最小二乗最適化器を実装しています。これは関連ノートブックに文書化されています： |lscolab|

.. |lscolab| image:: https://colab.research.google.com/assets/colab-badge.png
             :target: https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/python/least_squares.ipynb

.. _PyUSDexport:

USDエクスポーター
-----------------

`USDエクスポーター <https://github.com/google-deepmind/mujoco/tree/main/python/mujoco/usd>`__ モジュールにより、ユーザーはシーンと軌跡を `USDフォーマット <https://openusd.org/release/index.html>`__ で保存し、NVIDIA OmniverseやBlenderなどの外部レンダラーでレンダリングできます。これらのレンダラーは、デフォルトレンダラーでは提供されない、より高品質のレンダリング機能を提供します。さらに、USDへのエクスポートにより、ユーザーは異なるタイプのテクスチャマップを含めて、シーン内のオブジェクトをよりリアルに見せることができます。

.. _PyUSDInstallation:

インストール
^^^^^^^^^^^^

USDエクスポーターに必要な要件のインストールには、 `PyPI <https://pypi.org/project/mujoco/>`__ 経由が推奨されます：

.. code-block:: shell

   pip install mujoco[usd]

これにより、USDエクスポーターが必要とするオプションの依存関係 ``usd-core`` と ``pillow`` がインストールされます。

ソースからビルドする場合は、 `Pythonバインディングをビルド <https://mujoco.readthedocs.io/en/stable/python.html#building-from-source>`__ してください。次に、pipを使用して、必要な ``usd-core`` と ``pillow`` パッケージをインストールします。

.. _PyUSDExporter:

USDExporter
^^^^^^^^^^^

``mujoco.usd.exporter`` モジュールの ``USDExporter`` クラスにより、カスタムカメラとライトの定義に加えて、完全な軌跡の保存が可能になります。 ``USDExporter`` インスタンスのコンストラクタ引数は次のとおりです：

- ``model``：MjModelインスタンス。USDエクスポーターは、カメラ、ライト、テクスチャ、オブジェクトジオメトリの詳細を含む、モデルから関連情報を読み取ります。

- ``max_geom``：シーン内のジオムの最大数。内部 `mjvScene <https://mujoco.readthedocs.io/en/stable/APIreference/APItypes.html#mjvscene>`__ をインスタンス化する際に必要です。

- ``output_directory``：エクスポートされたUSDファイルとすべての関連アセットが格納されるディレクトリの名前。シーン/軌跡をUSDファイルとして保存するとき、エクスポーターは以下のディレクトリ構造を作成します。

  .. code-block:: text

      output_directory_root/
      └-output_directory/
        ├-assets/
        | ├-texture_0.png
        | ├-texture_1.png
        | └-...
        └─frames/
          └-frame_301.usd

  このファイル構造を使用することで、ユーザーは ``output_directory`` を簡単にアーカイブできます。USDファイル内のアセットへのすべてのパスは相対的であり、別のマシンでのUSDアーカイブの使用が容易になります。

- ``output_directory_root``：USD軌跡を追加するルートディレクトリ。

- ``light_intensity``：すべてのライトの強度。強度の単位はレンダラーによって異なる方法で定義される場合があるため、この値はレンダラー固有の基準で調整する必要がある場合があります。

- ``camera_names``：USDファイルに格納するカメラのリスト。各タイムステップで、定義された各カメラについて、その位置と向きを計算し、USDの指定されたフレームにその値を追加します。USDは複数のカメラを格納できます。

- ``verbose``：エクスポーターからログメッセージを出力するかどうか。

MJCFから直接ロードされたモデルをエクスポートしたい場合は、その方法を示す `デモ <https://github.com/google-deepmind/mujoco/blob/main/python/mujoco/usd/demo.py>`__ スクリプトを提供しています。このデモファイルは、USDエクスポート機能の例としても機能します。

.. _PyUSDBasicUsage:

基本的な使い方
^^^^^^^^^^^^^^

オプションの依存関係がインストールされると、USDエクスポーターは ``from mujoco.usd import exporter`` でインポートできます。

以下では、 ``USDExporter`` の使用の簡単な例を示します。初期化中に、 ``USDExporter`` は空のUSDステージと、まだ存在しない場合はassetsとframesディレクトリを作成します。さらに、モデルで定義された各テクスチャの.pngファイルを生成します。 ``update_scene`` が呼び出されるたびに、エクスポーターはシーン内のすべてのジオム、ライト、カメラの位置と向きを記録します。

``USDExporter`` は、フレームカウンタを維持することで内部的にフレームを追跡します。 ``update_scene`` が呼び出されるたびに、カウンタがインクリメントされ、対応するフレームのすべてのジオム、カメラ、ライトのポーズが保存されます。 ``update_scene`` を呼び出す前に、シミュレーションを複数回ステップできることに注意してください。最終的なUSDファイルには、最後の ``update_scene`` 呼び出し時のジオム、ライト、カメラのポーズのみが格納されます。

.. code-block:: python

    import mujoco
    from mujoco.usd import exporter

    m = mujoco.MjModel.from_xml_path('/path/to/mjcf.xml')
    d = mujoco.MjData(m)

    # USDExporterを作成
    exp = exporter.USDExporter(model=m)

    duration = 5
    framerate = 60
    while d.time < duration:

      # 物理をステップ
      mujoco.mj_step(m, d)

      if exp.frame_count < d.time * framerate:
        # 新しいフレームでUSDを更新
        exp.update_scene(data=d)

    # USDファイルをエクスポート
    exp.save_scene(filetype="usd")



.. _PyUSDExportAPI:

USD Export API
^^^^^^^^^^^^^^

- ``update_scene(self, data, scene_option)``：ユーザーが渡した最新のシミュレーションデータでシーンを更新します。この関数は、シーン内のジオム、カメラ、ライトを更新します。

- ``add_light(self, pos, intensity, radius, color, obj_name, light_type)``：指定されたプロパティを持つライトを事後的にUSDシーンに追加します。

- ``add_camera(self, pos, rotation_xyz, obj_name)``：指定されたプロパティを持つカメラを事後的にUSDシーンに追加します。

- ``save_scene(self, filetype)``：USDファイルタイプ拡張子 ``.usd``、 ``.usda``、または ``.usdc`` のいずれかを使用してUSDシーンをエクスポートします。

.. _PyUSDTodos:

欠けている機能
^^^^^^^^^^^^^^

以下に、USDエクスポーターの残りのアクションアイテムをリストします。追加のリクエストを提案するには、GitHubで新しい `機能リクエスト <https://github.com/google-deepmind/mujoco/issues/new/choose>`__ を作成してください。

- メタリック、オクルージョン、ラフネス、バンプなど、追加のテクスチャマップのサポートを追加。

- Isaacでのオンラインレンダリングのサポートを追加。

- カスタムカメラのサポートを追加。


.. _PyUtility:

ユーティリティ
==============

`python/mujoco <https://github.com/google-deepmind/mujoco/tree/main/python/mujoco>`__ ディレクトリにはユーティリティスクリプトも含まれています。


.. _PyMsh2obj:

msh2obj.py
----------

`msh2obj.py <https://github.com/google-deepmind/mujoco/blob/main/python/mujoco/msh2obj.py>`__ スクリプトは、表面メッシュ用の :ref:`レガシー.mshフォーマット<legacy-msh-docs>` （.mshを使用する体積メッシュの可能性がある :ref:`gmshフォーマット<gmsh-file-docs>` とは異なります）をOBJファイルに変換します。レガシーフォーマットは非推奨であり、将来のリリースで削除されます。すべてのレガシーファイルをOBJに変換してください。



.. _PyMjpy_migration:

mujoco-pyマイグレーション
=========================

mujoco-pyでは、主なエントリポイントは `MjSim <https://github.com/openai/mujoco-py/blob/master/mujoco_py/mjsim.pyx>`_ クラスです。ユーザーはMJCFモデルから状態を持つ ``MjSim`` インスタンスを構築し（ ``dm_control.Physics`` と同様）、このインスタンスは ``mjModel`` インスタンスとそれに関連する ``mjData`` への参照を保持します。対照的に、MuJoCo Pythonバインディング（ ``mujoco``）は、上記で説明したように、より低レベルのアプローチを取ります：Cライブラリの設計原則に従い、 ``mujoco`` モジュール自体はステートレスであり、単に基礎となるネイティブ構造体と関数をラップします。

mujoco-pyの完全な調査はこのドキュメントの範囲外ですが、以下では、mujoco-pyの特定の機能の非網羅的なリストに対する実装ノートを提供します：

``mujoco_py.load_model_from_xml(bstring)``
   このファクトリ関数は、状態を持つ ``MjSim`` インスタンスを構築します。 ``mujoco`` を使用する場合、ユーザーは :ref:`上記 <PyStructs>` で説明したようにファクトリ関数 ``mujoco.MjModel.from_xml_*`` を呼び出す必要があります。ユーザーは結果として得られる ``MjModel`` 構造体インスタンスを保持し、 ``mujoco.MjData(model)`` を呼び出して対応する ``MjData`` を明示的に生成する責任があります。

``sim.reset()``、 ``sim.forward()``、 ``sim.step()``
   ここでも上記と同様に、 ``mujoco`` ユーザーは、 ``MjModel`` と ``MjData`` のインスタンスを渡して、基礎となるライブラリ関数を呼び出す必要があります： :ref:`mujoco.mj_resetData(model, data) <mj_resetData>`、 :ref:`mujoco.mj_forward(model, data) <mj_forward>`、 :ref:`mujoco.mj_step(model, data) <mj_step>`。

``sim.get_state()``、 ``sim.set_state(state)``、 ``sim.get_flattened_state()``、 ``sim.set_state_from_flattened(state)``
   :ref:`Programming section <Simulation>` で説明されているように、MuJoCoライブラリの計算は特定の入力に対して決定論的です。mujoco-pyは、関連するフィールドの一部を取得および設定するメソッドを実装しています（同様に、 ``dm_control.Physics`` はフラット化されたケースに対応するメソッドを提供します）。この機能は :ref:`State and Control<siStateControl>` セクションで説明されています。

``sim.model.get_joint_qvel_addr(joint_name)``
   これは、このジョイントに対応する連続するインデックスのリストを返す、mujoco-pyの便利なメソッドです。リストは ``model.jnt_qposadr[joint_index]`` から始まり、その長さはジョイントタイプに依存します。  ``mujoco`` はこの機能を提供していませんが、このリストは ``model.jnt_qposadr[joint_index]`` と ``xrange`` を使用して簡単に構築できます。

``sim.model.*_name2id(name)``
   mujoco-pyは、 ``MjSim`` に、異なるタイプのオブジェクトのインデックスを効率的に検索できる辞書を作成します： ``site_name2id``、 ``body_name2id`` など。これらの関数は、関数 :ref:`mujoco.mj_name2id(model, type_enum, name) <mj_name2id>` を置き換えます。 ``mujoco`` は、エンティティ名を使用するための異なるアプローチ – :ref:`名前付きアクセス <PyNamed>` と、ネイティブの :ref:`mj_name2id` へのアクセスを提供します。

``sim.save(fstream, format_name)``
   これは、MuJoCoライブラリ（したがって ``mujoco`` も）がステートフルである唯一のコンテキストです：最後にコンパイルされたXMLのコピーをメモリに保持し、これは :ref:`mujoco.mj_saveLastXML(fname) <mj_saveLastXML>` で使用さ���ます。mujoco-pyの実装には便利な追加機能があり、ポーズ（ ``sim.data`` の状態によって決定）が保存前にモデルに追加されるキーフレームに変換されることに注意してください。この追加機能は現在 ``mujoco`` では利用できません。
