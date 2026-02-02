ファイル形式プラグイン
=========================

SdfFileFormatプラグインとは？
--------------------------------

OpenUSDフレームワークにおいて、 ``Sdf`` はScene Description Foundationsの略です。これは、シーンデータのシリアライゼーションと合成を処理する基礎レイヤーです。 ``SdfFileFormat`` プラグインは、USDに特定のファイル形式を読み書きする方法を教えるコンポーネントです。

デフォルトでは、USDは独自の形式（ ``.usda``、 ``.usdc``、 ``.usdz``）用のプラグインを備えており、コミュニティは `Adobe File Format Plugins
<https://github.com/adobe/USD-Fileformat-plugins/tree/main>`__ などのプラグイン拡張を作成しています。

MJCF ``SdfFileFormat`` プラグインにより、USD対応アプリケーションはMuJoCoのネイティブ ``.xml`` （MJCF）ファイルを、ネイティブUSDファイルであるかのように直接理解し、対話できるようになります。

何ができるようになるのか？
--------------------------------

このプラグインにより以下が可能になります：

1.  **USDでMJCFファイルを参照:** 標準のUSD合成アーク（参照、ペイロードなど）を使用して、MJCFファイルをより大きなUSDシーン内に直接含めることができます。例えば、 ``.xml`` ファイルで定義されたMuJoCoロボットを、USDでモデル化された部屋のシーンに配置できます。
2.  **USDツールでMJCFファイルを読み込む:** ``usdview`` やその他のUSDベースのアプリケーションは、MJCFファイルを開いて検査し、レンダリングでき、MJCF要素をUSDのprimと属性にオンザフライで変換します。
3.  **MJCFをUSDに変換:** このプラグインは、MJCFファイルを永続的なUSDファイル（例： ``.usda`` または ``.usdc``）に変換するための基礎として使用できます。

本質的に、これによりMJCFがUSDエコシステムのファーストクラスシチズンになります。

使用方法
------------------

1.  **インストール:** :doc:`building` を参照してください。

2.  **USDファイル（例： ``.usda``）での参照:**

    .. code-block:: usd
        :caption: example.usda

        #usda 1.0
        (
            upAxis = "Z"
        )

        def Xform "world"
        {
            def "robot" (
                prepend references = @./my_robot.xml@
            )
            {
            }
        }

    この例では、 ``my_robot.xml`` は同じディレクトリにあるMJCFファイルです。USDはプラグインを使用してその内容を読み込んで解釈します。

3.  **usdviewで開く:**

    .. code-block:: bash

       usdview my_robot.xml

    プラグインが正しく設定されていれば、 ``usdview`` はMJCFファイルで定義されたロボットをレンダリングします。

4.  **Pythonでの使用（USD APIを使用）:**

    .. code-block:: python

        from pxr import Usd

        # MJCFファイルをUSDステージとして読み込む
        stage = Usd.Stage.Open('my_robot.xml')

        if stage:
            print(f"Successfully opened {stage.GetRootLayer().identifier}")
            # 他のUSDステージと同様にステージを検査できる
            for prim in stage.TraverseAll():
                print(prim.GetPath())
        else:
            print("Failed to open MJCF file")

    このプラグインは、MuJoCoとUSDベースのワークフロー間の相互運用性を大幅に向上させ、MJCFで定義された物理アセットをより広い3D環境にシームレスに統合できるようにします。
