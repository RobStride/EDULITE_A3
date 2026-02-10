#!/usr/bin/env python3
"""
重力补偿标定结果分析和可视化工具

功能:
1. 加载标定结果 JSON 文件
2. 绘制力矩-角度曲线和拟合结果
3. 分析拟合质量
4. 生成报告

使用方法:
    python3 gravity_calibration_analyzer.py <calibration_result.json>
"""

import json
import numpy as np
import sys
import os

# 尝试导入可视化库
try:
    import matplotlib.pyplot as plt
    import matplotlib
    matplotlib.use('TkAgg')  # 使用 TkAgg 后端
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False
    print("警告: 未安装 matplotlib，将只显示文本报告")
    print("安装命令: pip3 install matplotlib")


def load_calibration_data(filepath: str) -> dict:
    """加载标定结果"""
    with open(filepath, 'r') as f:
        return json.load(f)


def print_text_report(data: dict):
    """打印文本报告"""
    print('\n' + '=' * 80)
    print('  EL-A3 重力补偿标定结果分析报告')
    print('=' * 80)
    
    print(f'\n标定时间: {data.get("timestamp", "未知")}')
    
    config = data.get('config', {})
    print(f'\n标定配置:')
    print(f'  采样角度数: {config.get("num_angles", "N/A")}')
    print(f'  每点采样数: {config.get("samples_per_angle", "N/A")}')
    print(f'  稳定时间: {config.get("settle_time", "N/A")} 秒')
    
    print(f'\n{"="*80}')
    print(f'{"关节":<10} {"sin_coeff":>12} {"cos_coeff":>12} {"offset":>12} {"RMSE":>10} {"R²":>8}')
    print('-' * 80)
    
    joints = data.get('joints', {})
    total_rmse = 0
    valid_count = 0
    
    for name in ['L1_joint', 'L2_joint', 'L3_joint', 'L4_joint', 'L5_joint', 'L6_joint']:
        if name in joints:
            j = joints[name]
            sin_c = j.get('sin_coeff', 0)
            cos_c = j.get('cos_coeff', 0)
            offset = j.get('offset', 0)
            rmse = j.get('rmse', 0)
            r2 = j.get('r_squared', 0)
            
            # 判断是否有有效数据
            raw = j.get('raw_data', {})
            has_data = len(raw.get('angles', [])) > 0
            
            if has_data:
                print(f'{name:<10} {sin_c:>12.4f} {cos_c:>12.4f} {offset:>12.4f} {rmse:>10.4f} {r2:>8.4f}')
                total_rmse += rmse
                valid_count += 1
            else:
                print(f'{name:<10} {"未标定":^50}')
    
    print('-' * 80)
    
    if valid_count > 0:
        print(f'\n平均 RMSE: {total_rmse/valid_count:.4f} Nm')
    
    # 生成 XACRO 参数
    print('\n' + '=' * 80)
    print('生成的 XACRO 参数:')
    print('-' * 80)
    
    for name in ['L1_joint', 'L2_joint', 'L3_joint', 'L4_joint', 'L5_joint', 'L6_joint']:
        if name in joints:
            j = joints[name]
            raw = j.get('raw_data', {})
            if len(raw.get('angles', [])) > 0:
                short_name = name.replace('_joint', '')
                print(f'<param name="gravity_comp_{short_name}_sin">{j.get("sin_coeff", 0):.4f}</param>')
                print(f'<param name="gravity_comp_{short_name}_cos">{j.get("cos_coeff", 0):.4f}</param>')
                print(f'<param name="gravity_comp_{short_name}_offset">{j.get("offset", 0):.4f}</param>')
    
    print('=' * 80)


def plot_calibration_results(data: dict, save_path: str = None):
    """绘制标定结果"""
    if not HAS_MATPLOTLIB:
        print("无法绘图: matplotlib 未安装")
        return
    
    joints = data.get('joints', {})
    joint_names = ['L1_joint', 'L2_joint', 'L3_joint', 'L4_joint', 'L5_joint', 'L6_joint']
    
    # 统计有效关节数
    valid_joints = []
    for name in joint_names:
        if name in joints:
            raw = joints[name].get('raw_data', {})
            if len(raw.get('angles', [])) > 0:
                valid_joints.append(name)
    
    if len(valid_joints) == 0:
        print("没有有效的标定数据可供绘图")
        return
    
    # 确定子图布局
    n_plots = len(valid_joints)
    n_cols = min(3, n_plots)
    n_rows = (n_plots + n_cols - 1) // n_cols
    
    fig, axes = plt.subplots(n_rows, n_cols, figsize=(5*n_cols, 4*n_rows))
    fig.suptitle('EL-A3 重力补偿标定结果', fontsize=14, fontweight='bold')
    
    # 确保 axes 是二维数组
    if n_plots == 1:
        axes = np.array([[axes]])
    elif n_rows == 1:
        axes = axes.reshape(1, -1)
    elif n_cols == 1:
        axes = axes.reshape(-1, 1)
    
    for idx, name in enumerate(valid_joints):
        row = idx // n_cols
        col = idx % n_cols
        ax = axes[row, col]
        
        j = joints[name]
        raw = j.get('raw_data', {})
        angles = np.array(raw.get('angles', []))
        torques = np.array(raw.get('torques', []))
        torque_stds = np.array(raw.get('torque_stds', []))
        
        sin_c = j.get('sin_coeff', 0)
        cos_c = j.get('cos_coeff', 0)
        offset = j.get('offset', 0)
        rmse = j.get('rmse', 0)
        r2 = j.get('r_squared', 0)
        
        # 绘制原始数据点（带误差棒）
        if len(torque_stds) > 0 and np.any(torque_stds > 0):
            ax.errorbar(np.degrees(angles), torques, yerr=torque_stds, 
                       fmt='o', color='blue', markersize=6, capsize=3,
                       label='测量数据', alpha=0.7)
        else:
            ax.scatter(np.degrees(angles), torques, color='blue', s=50,
                      label='测量数据', alpha=0.7)
        
        # 绘制拟合曲线
        angles_fit = np.linspace(angles.min(), angles.max(), 100)
        torques_fit = sin_c * np.sin(angles_fit) + cos_c * np.cos(angles_fit) + offset
        ax.plot(np.degrees(angles_fit), torques_fit, 'r-', linewidth=2, 
               label=f'拟合曲线')
        
        # 设置标签和标题
        short_name = name.replace('_joint', '')
        ax.set_title(f'{short_name}\n'
                    f'τ = {sin_c:.3f}sin(θ) + {cos_c:.3f}cos(θ) + {offset:.3f}\n'
                    f'RMSE={rmse:.4f}, R²={r2:.4f}',
                    fontsize=10)
        ax.set_xlabel('角度 (°)')
        ax.set_ylabel('力矩 (Nm)')
        ax.legend(loc='best', fontsize=8)
        ax.grid(True, alpha=0.3)
        ax.axhline(y=0, color='gray', linestyle='--', alpha=0.5)
        ax.axvline(x=0, color='gray', linestyle='--', alpha=0.5)
    
    # 隐藏多余的子图
    for idx in range(len(valid_joints), n_rows * n_cols):
        row = idx // n_cols
        col = idx % n_cols
        axes[row, col].set_visible(False)
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f'\n图表已保存到: {save_path}')
    
    plt.show()


def plot_residuals(data: dict, save_path: str = None):
    """绘制残差分析图"""
    if not HAS_MATPLOTLIB:
        return
    
    joints = data.get('joints', {})
    joint_names = ['L1_joint', 'L2_joint', 'L3_joint', 'L4_joint', 'L5_joint', 'L6_joint']
    
    # 收集所有残差
    all_residuals = []
    joint_residuals = {}
    
    for name in joint_names:
        if name in joints:
            j = joints[name]
            raw = j.get('raw_data', {})
            angles = np.array(raw.get('angles', []))
            torques = np.array(raw.get('torques', []))
            
            if len(angles) > 0:
                sin_c = j.get('sin_coeff', 0)
                cos_c = j.get('cos_coeff', 0)
                offset = j.get('offset', 0)
                
                predicted = sin_c * np.sin(angles) + cos_c * np.cos(angles) + offset
                residuals = torques - predicted
                
                all_residuals.extend(residuals)
                joint_residuals[name] = residuals
    
    if len(all_residuals) == 0:
        print("没有数据用于残差分析")
        return
    
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    fig.suptitle('残差分析', fontsize=14, fontweight='bold')
    
    # 残差直方图
    ax1 = axes[0]
    ax1.hist(all_residuals, bins=20, edgecolor='black', alpha=0.7)
    ax1.axvline(x=0, color='red', linestyle='--', linewidth=2)
    ax1.set_xlabel('残差 (Nm)')
    ax1.set_ylabel('频次')
    ax1.set_title(f'残差分布\n均值={np.mean(all_residuals):.4f}, 标准差={np.std(all_residuals):.4f}')
    ax1.grid(True, alpha=0.3)
    
    # 各关节残差箱线图
    ax2 = axes[1]
    labels = []
    data_to_plot = []
    for name in joint_names:
        if name in joint_residuals:
            labels.append(name.replace('_joint', ''))
            data_to_plot.append(joint_residuals[name])
    
    if len(data_to_plot) > 0:
        bp = ax2.boxplot(data_to_plot, labels=labels)
        ax2.axhline(y=0, color='red', linestyle='--', linewidth=1)
        ax2.set_xlabel('关节')
        ax2.set_ylabel('残差 (Nm)')
        ax2.set_title('各关节残差分布')
        ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    if save_path:
        base, ext = os.path.splitext(save_path)
        residual_path = base + '_residuals' + ext
        plt.savefig(residual_path, dpi=150, bbox_inches='tight')
        print(f'残差分析图已保存到: {residual_path}')
    
    plt.show()


def main():
    if len(sys.argv) < 2:
        # 查找最新的标定文件
        script_dir = os.path.dirname(os.path.abspath(__file__))
        json_files = [f for f in os.listdir(script_dir) 
                     if f.startswith('gravity_calibration_') and f.endswith('.json')]
        
        if json_files:
            # 按时间排序，取最新的
            json_files.sort(reverse=True)
            filepath = os.path.join(script_dir, json_files[0])
            print(f'使用最新的标定文件: {json_files[0]}')
        else:
            print('用法: python3 gravity_calibration_analyzer.py <calibration_result.json>')
            print('\n或将标定结果文件放在 scripts 目录下')
            return
    else:
        filepath = sys.argv[1]
    
    if not os.path.exists(filepath):
        print(f'错误: 文件不存在: {filepath}')
        return
    
    # 加载数据
    print(f'\n加载标定数据: {filepath}')
    data = load_calibration_data(filepath)
    
    # 打印文本报告
    print_text_report(data)
    
    # 绘制图表
    if HAS_MATPLOTLIB:
        print('\n正在生成可视化图表...')
        
        # 生成保存路径
        base_name = os.path.splitext(os.path.basename(filepath))[0]
        save_dir = os.path.dirname(filepath)
        save_path = os.path.join(save_dir, f'{base_name}_plot.png')
        
        plot_calibration_results(data, save_path)
        plot_residuals(data, save_path)
    else:
        print('\n提示: 安装 matplotlib 可以查看可视化图表')
        print('  pip3 install matplotlib')


if __name__ == '__main__':
    main()
