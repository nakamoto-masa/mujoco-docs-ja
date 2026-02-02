# https://just.systems

# MuJoCoドキュメント（日本語版）を開く
ja:
    open docs/_build/html/index.html

# MuJoCoドキュメント（英語版・公式）を開く
en:
    open https://mujoco.readthedocs.io/

# Sphinxドキュメントを通常ビルド
[working-directory: 'docs']
build:
    uv run sphinx-build . _build/html

# Sphinxドキュメントをキャッシュクリアしてビルド（-E オプション）
[working-directory: 'docs']
rebuild:
    uv run sphinx-build -E . _build/html
