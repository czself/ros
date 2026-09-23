# 人物立牌插入包

这里放置 18 张人物图片和一个立牌模板，暂不绑定场景位置。

尺寸严格按要求：

- 高：`0.15 m`（15 cm）
- 宽：`0.045 m`（4.5 cm）
- 厚：`0.005 m`（0.5 cm）

图片编号：`person_01.png` 到 `person_16.png`，以及 `person_F1.png`、`person_F2.png`。

## 使用

1. 将 `person_standee_template.sdf` 复制为一个独立模型。
2. 在模型材质中把 `PersonStandee/Person01` 和 `person_01.png` 改成对应编号。
3. 在 world 中通过 `<include>` 插入模型并自行填写 `<pose>x y 0 0 0 yaw</pose>`。

模板使用 0.045 x 0.005 x 0.15 m 的无图案底板，并在朝向 -Y 的正面增加一层 0.1 mm 图片面；背面不贴图。碰撞体仍覆盖完整立牌尺寸，不会改变地图尺寸或人物比例。
