import pandas as pd
import matplotlib.pyplot as plt
import sys

file_path = sys.argv[1]
 # 读取文本文件并转换为DataFrame对象
df = pd.read_csv(file_path, delimiter=' ', header=None)
df.dropna(inplace=True)
 # 得到df的行数与列数
rows = df.shape[0]
cols = df.shape[1]
print("行数:", rows)
print("列数:", cols)
 # 计算相邻行中第一列数据的差值
df['seq_column_diff'] = df[0].diff()
df['ts_column_diff'] = df[1].diff()
ts_mean_value = df['ts_column_diff'].mean()
seq_mean_value = df['seq_column_diff'].mean()
print("序列号戳之差均值:", seq_mean_value)
print("时间戳之差均值:", ts_mean_value)

fig, ax = plt.subplots(3, 1, figsize=(8, 6))
ax[0].plot(df.index, df['seq_column_diff'])
ax[0].set_ylabel('seq_column_diff')

if cols == 2:
    ax[1].plot(df[0], df[1])
    ax[1].set_ylabel('value')
else:
    ax[1].plot(df.index, df['ts_column_diff'])
    ax[1].set_ylabel('ts_column_diff')

# if cols > 2:
#     fig, axs = plt.subplots(cols - 1, 1, sharex=True)
#     for i in range(1, cols):
#         print(i)
#         axs[i - 1].plot(df[0], df[i])
#     plt.xlabel('Row')
fig, axs = plt.subplots(3, 1, sharex=True)
# odom pack使用
# axs[0].plot(df[1], df[9]) # yawrate
# axs[1].plot(df[1], df[10]) # source
# axs[2].plot(df[1], df[11]) # err code

# imu pack
# axs[0].plot(df[0], df[4], marker='o', markersize=2) # ax
# axs[1].plot(df[0], df[5], marker='o', markersize=2) # ay
# axs[2].plot(df[0], df[7], marker='o', markersize=2) # az
# axs[0].plot(df[0], df[7]) # ax
# axs[1].plot(df[0], df[5]) # ay
# axs[2].plot(df[0], df[6]) # az

# bias
# axs[1].plot(df[0], df[5])

# axs[0].plot(df[0], df[3]) # ax
# df_0_diff = df[0].diff()
# axs[0].plot(df[0], df[1].diff(), marker='o', markersize=2) # ay
# axs[1].plot(df[0], df[4].diff(), marker='o', markersize=2) # ay

axs[0].plot(df[0], df[1], marker='o', markersize=2) # ay
axs[1].plot(df[0], df[2], marker='o', markersize=2) # ay
axs[2].plot(df[0], df[3], marker='o', markersize=2) # ax
# axs[1].plot(df[0], df[4], marker='o', markersize=2) # ay


# axs[0].plot(df[0], df[9], marker='o', markersize=2) # 回转编码器
# axs[1].plot(df[0], df[10], marker='o', markersize=2) #
# axs[2].plot(df[0], df[11], marker='o', markersize=2) #


axs[0].grid(True)
axs[1].grid(True)
axs[2].grid(True)


plt.show()
