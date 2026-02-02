エクスポート
============

.. WARNING:: OpenUSDサポートは現在実験的なものであり、頻繁に変更される可能性があります。

現在、MuJoCoシーンをOpenUSD形式にエクスポートすることは、活発に開発中の領域です。エクスポートの主な方法は、Python APIを通じて提供される予定です。

USDExporter
-----------

現時点では、MuJoCoからUSDをエクスポートする唯一の方法は、 :doc:`../python` の既存のUSDExporterを使用することです。

既存のUSDシーンにシミュレーションをアニメーションとして書き込むためのネイティブサポートに取り組んでいます。更新情報はこちらで確認してください。

mujoco-usd-converter
--------------------

既存のMJCFアセットを強力なオーサリングガイドラインに従ってUSDに変換するには、 `Newton
mujoco-usd-converter <https://github.com/newton-physics/mujoco-usd-converter>`__ を推奨します。これらのアセットは元のMJCFアセットへの参照を持ちませんが、 :doc:`mjcPhysics <mjcPhysics>` スキーマを使用してソースアセットの忠実な表現を可能にします。

これらのアセットを開くと、 :doc:`MJCFファイル形式プラグイン
<mjcf_file_format_plugin>` によって生成されたものと似ていますが、MJCFからUSDへの変換による実行時のオーバーヘッドはありません。
