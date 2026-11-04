# 断言写法模板

## 1. 基础：`chk` 任务 + 汇总

```verilog
integer fails = 0, checks = 0;

task chk;
    input [8*80:1] name;      // 名字一律 ASCII（%0s 打中文会乱码）
    input integer  got;
    input integer  exp;
    begin
        checks = checks + 1;
        if (got !== exp) begin
            $display("  *** FAIL  %0s : got %0d, expect %0d", name, got, exp);
            fails = fails + 1;
        end else
            $display("   ok       %0s = %0d", name, got);
    end
endtask

// 结尾
if (fails == 0) $display("  ALL PASS  (%0d checks)", checks);
else            $display("  FAILED    %0d of %0d checks", fails, checks);
```

**`got !== exp`（四态比较）**：不要用 `!=`。X 值用 `!=` 会漏，用 `!==` 才能抓出
"信号根本没驱动"这种错。

---

## 2. 不变量断言（首选）

不要写死绝对坐标，写"关系"。这样改了布局常量断言依然有效，而且失败信息更有指向性。

```verilog
// 弹层：选中项必须压在 ComboBox 那一行 + 32 等距 + 面板 6/6 内边距
task chk_pop;
    input integer sel;
    input integer cy;                 // 该行控件顶边
    integer base;
    begin
        base = cy - sel*32 - 2;
        chk("sel item on combo row", dut.u_gfx.u_mem.cell_r[15+sel][8:0], cy - 2);
        chk("item0 y", dut.u_gfx.u_mem.cell_r[15][8:0], base);
        chk("item1 y", dut.u_gfx.u_mem.cell_r[16][8:0], base + 32);
        chk("item2 y", dut.u_gfx.u_mem.cell_r[17][8:0], base + 64);
        chk("panel top", dut.u_gfx.u_mem.cell_r[4][8:0],   base - 6);
        chk("panel bot", dut.u_gfx.u_mem.cell_r[4][27:19], base + 98);
    end
endtask

// 被弹层盖住的行必须不画，其余行必须在原位
task chk_rows;
    integer i;
    begin
        for (i = 0; i < 5; i = i + 1)
            if ((i == dut.u_menu.pop_wx0) || (i == dut.u_menu.pop_wx1))
                chk("row hidden", dut.u_gfx.u_mem.cell_r[9+i][18:9], 0);
            else
                chk("row shown",  dut.u_gfx.u_mem.cell_r[9+i][8:0], 60 + 80*i);
    end
endtask

// 任意两条文字行的 16px 字格不许相交（一条断言守一整类问题）
task chk_txt_noclash;
    integer i, j, yi, yj, bad;
    begin
        bad = 0;
        for (i = 0; i < 8; i = i + 1)
            for (j = i + 1; j < 8; j = j + 1) begin
                yi = dut.u_menu.txt_xy[i*32+16 +: 16];
                yj = dut.u_menu.txt_xy[j*32+16 +: 16];
                if (!((yi == yj) || (yi + 16 <= yj) || (yj + 16 <= yi))) begin
                    bad = bad + 1;
                    $display("    (line%0d y=%0d vs line%0d y=%0d overlap)", i, yi, j, yj);
                end
            end
        chk("txt cells never clash", bad, 0);
    end
endtask
```

注意 `chk_txt_noclash` 是**汇总成一条**的：28 对组合各出一条 `ok` 会把日志刷花。

---

## 3. 文字监视器（抓"这行到底写了什么字"）

有些 bug 只看信号量不出来 —— 比如"系统页改一个 dropdown 所有 dropdown 都变"，
值寄存器全对，错在**写进字库的字符**。这时要监听 `str_we/str_line/str_idx/str_code`：

```verilog
reg [15:0] cap14 [0:7];        // 每行第 14 列最后写入的字符码
reg [15:0] cap15 [0:7];        // 第 15 列（勾选框"开/关"、Button 的"行"）
reg [15:0] cap00 [0:7];        // 第 0 列（标签首字，验"被遮住的行有没有清空"）

always @(posedge clk) begin
    if (dut.u_menu.str_we) begin
        if (dut.u_menu.str_idx == 8'd0)  cap00[dut.u_menu.str_line] <= dut.u_menu.str_code;
        if (dut.u_menu.str_idx == 8'd14) cap14[dut.u_menu.str_line] <= dut.u_menu.str_code;
        if (dut.u_menu.str_idx == 8'd15) cap15[dut.u_menu.str_line] <= dut.u_menu.str_code;
    end
end

function [15:0] optcode;       // 低/中/高
    input integer k;
    begin
        optcode = (k <= 0) ? 16'hB5CD : (k == 1) ? 16'hD6D0 : 16'hB8DF;
    end
endfunction

// 用法：打开帧率的下拉，验证分辨率那一行仍是自己的值
chk("resolution shows own value", cap14[2], optcode(dut.u_menu.val1));
chk("covered row blanked",        cap00[2], 0);
```

GB2312 码用 Python 现查，别凭记忆：

```bash
$PY -c "print('执行'.encode('gb2312').hex().upper())"   # D6B4D0D0
```

---

## 4. `cell_r` 位布局（属性断言必查）

`osd_cellmem` 每槽 63 位 = `attr[23:0] + geo[38:0]`：

```
cell_r[62:59]  type        0=Solid 1=Box 2=Panel 4=TrackBar 5=CheckBox
                           6=ComboBox 7=Button 8=ProgressBar 9=分隔线
                           10=NumericUpDown 11=游标框
cell_r[58:55]  style       样式表下标
cell_r[54:51]  rad         圆角半径（0 = 直角，判角只在 rad≠0 时生效）
cell_r[50:49]  mask        圆角外侧继承哪一层：0=面板 1=遮罩 2=弹层底板
cell_r[48:39]  v0
cell_r[37:28]  gx1
cell_r[27:19]  gy1
cell_r[18:9]   gx0
cell_r[8:0]    gy0
```

**踩过的坑**：`geo` 是 `{en, x1[9:0], y1[8:0], x0[9:0], y0[8:0]}` 共 39 位，
最高位是 `en`。所以 x1 是 `[37:28]`，不是 `[38:29]`（后者是 `{en, x1[9:1]}`，
读出来是 `512 + x1/2`）。写断言前先拿一个已知值试一下。

---

## 5. 按键与节拍

```verilog
task kpress;                    // 单拍脉冲（真实工程里按键就是单拍）
    input [3:0] k;
    begin
        @(negedge clk); key_press = k;
        @(negedge clk); key_press = 4'd0;
        repeat (6) @(posedge clk);
    end
endtask

task lay;                       // 排一帧：ps_start 一个高电平 + 等布局状态机跑完
    begin
        @(negedge clk); ps_start = 1'b1;
        repeat (8) @(posedge clk);
        @(negedge clk); ps_start = 1'b0;
        repeat (12) @(posedge clk);
    end
endtask

task lays; input integer n; integer i;
    begin for (i = 0; i < n; i = i + 1) lay(); end
endtask
```

- 改状态后至少 `lays(22)`（6 条动画通道各 1 拍 + 布局多段）；只排 2 帧是**故意**
  用来验"动画中途"的行为（例如"换行不能改显示值"）。
- 按键编码：`KEY1=4'h1` 开/返回、`KEY2=4'h2` 进入/确认、`KEY3=4'h4` 下、`KEY4=4'h8` 上。

---

## 6. 断言命名规范

`<段号><序号> <一句话>`，例如 `H row0 visible`、`U7 pop_a clamped`、`V8 resolution shows its own value`。
段号让人能一眼定位到测试台的分段，句子在失败时直接说明"什么东西不对"。
