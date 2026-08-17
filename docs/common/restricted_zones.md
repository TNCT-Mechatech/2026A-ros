# Restricted zones

フィールド内部の任意の単純多角形を、LiDAR反射点の無視ゾーンおよびA*の進入禁止ゾーンとして設定できます。通常のOccupancyGridは変更しないため、外周フェンスは従来どおり地図上の壁として残り、LiDARによるMCLにも利用されます。

サンプルの設定ファイルは `natto_nhk2025_sample/config/map/restricted_zones.yaml` です。デフォルトでは機能が無効になり、MCLとA*は従来どおり動作します。実際のゾーン座標が決まったら、次の形式で同ファイルまたは別のYAMLへ設定してください。

```yaml
/**:
  ros__parameters:
    restricted_zone_names:
      - example_zone

    restricted_zones.example_zone.points:
      - 1.0
      - 2.0
      - 2.0
      - 2.0
      - 2.0
      - 3.0
      - 1.0
      - 3.0

    restricted_zones.example_zone.lidar_ignore: true
    restricted_zones.example_zone.keepout: true
```

- `restricted_zone_names`: 読み込むゾーン名の配列です。空なら全ゾーンが無効です。ROS 2 Jazzyはparam file内のリテラル `[]` の要素型を決定できずNode生成前に失敗するため、同梱YAMLでは空文字1要素を型付き空配列の代替として使用し、ローダーが0件として扱います。ゾーンを設定する場合は通常の非空string配列を使用します。
- `points`: `[x1, y1, x2, y2, ...]` の順に3頂点以上を並べます。単位はm、座標系は `map` です。時計回り・反時計回りの両方に対応し、辺上と頂点上もゾーン内として扱います。
- `lidar_ignore`: `true` の場合、各MCLパーティクルで `map` 座標へ変換した反射点がゾーン内にあるとき、その点だけを尤度計算から除外します。光線がゾーンを通過して奥の外周フェンスで反射した場合、その反射点はゾーン外なので除外しません。
- `keepout`: `true` の場合、ゾーン内部と境界をA*用障害物として塗り、通常の壁と一緒に `xy_inflation` と `xy_offset` を適用します。公開されるcostmapにも反映されます。

`lidar_ignore` と `keepout` は独立しており、4通りの組み合わせを使用できます。不正なpointsを持つゾーンや重複名は警告してそのゾーンだけを無効化します。

赤・青とも自陣左下を `map` の `(0, 0)` とするため、同じ座標設定を利用できます。この機能独自の座標反転は行わず、既存の `reverse_y` の挙動も変更しません。別の設定ファイルを使う場合は、`natto.launch.xml` の `restricted_zones_param_path` にインストール後のパスを渡してください。
