import sensor, image, lcd, time
from Maix import utils
from machine import UART
from fpioa_manager import fm

# カメラの初期化と設定
sensor.reset()
sensor.set_pixformat(sensor.RGB565)  # RGB565フォーマット
sensor.set_framesize(sensor.QVGA)    # 320x240解像度

#sensor.set_auto_gain(False)         # オートゲインをオフ
# sensor.set_auto_whitebal(False)   # オートホワイトバランスをオフ 画像全体が黄色っぽくなってしまう
#sensor.set_auto_whitebal(False, rgb_gain_db = (100, 100, 100)) # ゲインを設定しておく

sensor.set_brightness(0)            # 明度の調整
sensor.set_saturation(0)            # 彩度の調整
sensor.set_contrast(3)              # コントラストの調整

sensor.set_vflip(1)          # 画像反転

sensor.skip_frames(time=2000)        # カメラ初期化のための待機
sensor.run(1)


# シリアル通信の初期化
fm.register(35,fm.fpioa.UART1_TX, force=True)
fm.register(34,fm.fpioa.UART1_RX, force=True)
uart = UART(UART.UART1, 115200, 8, None, 1, timeout=1000, read_buf_len=4096)


#uart = utils.UART(3, 115200)

def mask_image_and_get_color_and_center(img):
    # 白い背景をマスクする
    thresholds = (0, 100, -64, 18, -33, 50)  # RGBのしきい値
    #mask = img.binary([thresholds], invert=False)
    #masked_image = img.add(mask)

    blobs = img.find_blobs([thresholds], invert=True, pixels_threshold =100) # しきい値外の色を検出
    if blobs:

        # 最大サイズのブロブを見つける
        largest_blob = max(blobs, key=lambda b: b.pixels())
        img.draw_rectangle(largest_blob[0:4],color=(0, 255, 0)) # 検出した色を矩形で囲む
        img.draw_cross(largest_blob[5], largest_blob[6], color=(0, 255, 0)) # 検出した色の中心に十字を描く


        color_counts = {}

        # 最大ブロブのROIを取得
        roi = largest_blob.rect()
        # ROI内の全ピクセルを走査
        stride = 3
        for y in range(roi[1], roi[1] + roi[3], stride):
            for x in range(roi[0], roi[0] + roi[2], stride):
                if img.get_pixel(x, y) != 0:  # 有効ピクセル（ゼロ以外）
                    r, g, b = img.get_pixel(x, y)
                    # RGB値を4ビットシフトしてからカウント
                    if not(r==0 and g==255 and b==0):
                        #print(r,end=' , ')
                        #print(g,end=' , ')
                        #print(b,end=' , ')

                        color = (r >> 4, g >> 4, b >> 4)
                        if color in color_counts:
                            color_counts[color] += 1
                        else:
                            color_counts[color] = 1
                        #print(color)
        #print("-----------------------------------------------------------")

        if color_counts:
            # 最も頻繁に出現する色（モード）を取得
            mode_color = max(color_counts, key=color_counts.get)
            print("Detected ball color (RGB):", (mode_color[0], mode_color[1], mode_color[2]))

            data_packet = bytearray([250, mode_color[0], mode_color[1], mode_color[2]])
            uart.write(data_packet)
        else:
            print("Detected ball color (RGB):", (0,0,0))
            data_packet = bytearray([250,15,15,15])
            uart.write(data_packet)

    else:
        print("nothing")
        data_packet = bytearray([250,15,15,15])
        uart.write(data_packet)


    return 0


    # マスクされた領域のピクセルを抽出
    #r_sum, g_sum, b_sum, x_sum, y_sum, count = 0, 0, 0, 0, 0, 0
    #for x in range(mask.width()):
        #for y in range(mask.height()):
            #if mask.get_pixel(x, y) != 0:  # マスクされたピクセルをチェック
                #r, g, b = img.get_pixel(x, y)
                #r_sum += r
                #g_sum += g
                #b_sum += b
                #x_sum += x
                #y_sum += y
                #count += 1

    #if count > 0:
        #avg_r = r_sum // count
        #avg_g = g_sum // count
        #avg_b = b_sum // count
        #center_x = x_sum // count
        #center_y = y_sum // count
        #return avg_r, avg_g, avg_b, center_x, center_y
    #else:
        #return 0, 0, 0, -1, -1  # ボールが見つからなかった場合

while True:
    img = sensor.snapshot()  # カメラから画像を取得
    #avg_r, avg_g, avg_b, center_x, center_y =
    mask_image_and_get_color_and_center(img)  # 画像をマスクして色と中心を取得

    time.sleep(0.1)  # 少し待機してループを繰り返す
