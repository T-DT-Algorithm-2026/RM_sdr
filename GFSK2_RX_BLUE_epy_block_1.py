import numpy as np
from gnuradio import gr
from scipy import signal

class rm_gfsk_score_monitor(gr.sync_block):
    """
    RoboMaster 2-GFSK 相关性打分监视器 (输出连续的匹配波形)
    """
    def __init__(self, sps=52):
        gr.sync_block.__init__(
            self,
            name="RM GFSK Score Monitor",
            in_sig=[np.float32],
            out_sig=[np.float32] # 输出连续的 Float32 波形
        )
        self.sps = sps
        
        # 生成 Access Code 理想波形
        access_code_hex = 0x2F6F4C74B914492E
        bits = format(access_code_hex, '064b')
        symbols = np.array([1.0 if b == '1' else -1.0 for b in bits], dtype=np.float32)
        self.kernel = np.repeat(symbols, self.sps)
        self.kernel_len = len(self.kernel)
        
        # 让底层的 C++ 帮我们完美管理滑动窗口
        self.set_history(self.kernel_len)

    def work(self, input_items, output_items):
        in0 = input_items[0]
        out = output_items[0]
        
        # --- 增加：和接收端一模一样的预处理逻辑 ---
        # 1. 动态去直流
        centered_in0 = in0 - np.mean(in0)
        
        # 2. 极性二值化 (强制压平为 +1 和 -1)
        sign_in0 = np.sign(centered_in0)
        sign_in0[sign_in0 == 0] = 1.0
        
        # 3. 使用二值化后的波形算得分
        corr = signal.correlate(sign_in0, self.kernel, mode='valid', method='fft')
        
        out[:] = corr
        return len(out)
