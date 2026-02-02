インポート
=========

.. WARNING:: OpenUSDサポートは現在実験的なものであり、頻繁に変更される可能性があります。

MuJoCoはOpenUSDファイル（ ``.usd``、 ``.usda``、 ``.usdc``、 ``.usdz``）からアセットを読み込むことができます。これにより、USDで定義されたアセットやシーンをMuJoCoシミュレーションに組み込むことができます。

MJCFでのUSD
-----------------------------

USDを有効にしてmujocoをビルドした場合、 ``<model`` タグとコンテンツタイプ ``text/usd`` を使用して、MJCFからUSDアセットを参照できます。

.. code-block:: xml
    :caption: example.xml

    <mujoco>
      <asset>
        <model file="chair.usdz" name="chair" content_type="text/usd"/>
      </asset>

      <worldbody>
        ...
      </worldbody>
    </mujoco>

この例では、 ``<asset>`` 内の ``<model file="chair.usdz"/>`` 行がMuJoCoにUSDファイルを読み込んで処理するよう指示しています。
