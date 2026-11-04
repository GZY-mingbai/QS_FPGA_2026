//==============================================================================
//  key_router.v —— 板载 4 键 + 五向键 → 统一的 4 位按键总线
//==============================================================================
//
//  【总线位序】（与 App / osd_menu / zoom_osd 原有约定一致）
//      bit0 = KEY1（开菜单 / 返回 / 退出缩放）
//      bit1 = KEY2（确认 / 进入缩放 / 缩放里切功能）
//      bit2 = KEY3（下一张 / 下移 / 放大）
//      bit3 = KEY4（上一张 / 上移 / 缩小）
//
//  【板载 4 键】恒等 OR 进总线，一位不改 ⇒ 原有行为零变化。
//
//  【五向键按上下文 3 选 1】（优先级：缩放 > 菜单 > 查看器；菜单与缩放互斥）
//      五向键 │ 查看器        │ 缩放界面          │ 菜单
//      ───────┼──────────────┼──────────────────┼──────────────
//      UP     │ KEY2 进缩放   │ KEY4 左/上/缩小   │ KEY4 上移
//      DOWN   │ KEY2 进缩放   │ KEY3 右/下/放大   │ KEY3 下移
//      LEFT   │ KEY4 上一张   │ KEY2 切功能       │ KEY1 返回
//      RIGHT  │ KEY3 下一张   │ KEY2 切功能       │ KEY2 确认
//      OK     │ KEY1 开菜单   │ KEY1 退出         │ KEY2 确认
//
//  【电平（长按连发用）】只有 osd_menu 在"菜单开 + 编辑中"读 key_level[3:2]，
//    此时 UP→KEY4、DOWN→KEY3 ⇒ 直接把 UP/DOWN 的电平并进 bit3/bit2 即可，
//    不需要看上下文（别的上下文没有读者）。OK/LEFT/RIGHT 不驱动电平。
//
//  【时序】纯组合。press 是 clk 域单拍脉冲（与原 4 键相同）；menu_on / zoom_on
//    是 sd_card_clk 域的慢变电平，只用来门控，和原来 key_press 直接进 App 同性质。
//==============================================================================
module key_router (
    input  wire [3:0] hw_press,     // 板载 4 键按下沿 {K4,K3,K2,K1}
    input  wire [3:0] hw_level,     // 板载 4 键电平（1 = 松开）
    input  wire [4:0] bt_press,     // 五向键按下沿 {OK,DN,RT,LT,UP}
    input  wire [4:0] bt_level,     // 五向键电平（1 = 松开）
    input  wire       menu_on,      // OSD 菜单开着（含文件页）
    input  wire       zoom_on,      // 缩放界面开着
    output wire [3:0] key_press,
    output wire [3:0] key_level
);

wire up = bt_press[0];
wire lt = bt_press[1];
wire rt = bt_press[2];
wire dn = bt_press[3];
wire ok = bt_press[4];

wire c_zoom = zoom_on;
wire c_menu = menu_on & ~zoom_on;
wire c_view = ~menu_on & ~zoom_on;

wire [3:0] bt_map;
assign bt_map[0] = (ok & ~c_menu) | (lt & c_menu);
assign bt_map[1] = (c_view & (up | dn)) | (c_zoom & (lt | rt)) | (c_menu & (rt | ok));
assign bt_map[2] = (rt & c_view) | (dn & ~c_view);
assign bt_map[3] = (lt & c_view) | (up & ~c_view);

assign key_press = hw_press | bt_map;
assign key_level = hw_level & {bt_level[0], bt_level[3], 2'b11};

endmodule
