#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
多车编队仿真演示脚本
Formation Simulation Demo Script
"""

import subprocess
import sys
import time
import os

def print_banner():
    """打印欢迎横幅"""
    print("=" * 60)
    print("🚁 多车编队仿真系统演示")
    print("Multi-Vehicle Formation Simulation Demo")
    print("=" * 60)
    print()

def print_menu():
    """打印菜单选项"""
    print("请选择演示场景:")
    print("1. 直线编队 (4辆车)")
    print("2. V字编队 (6辆车)")
    print("3. 菱形编队 (8辆车)")
    print("4. 圆形编队 (6辆车)")
    print("5. 大型编队演示 (10辆车)")
    print("6. 自定义参数")
    print("0. 退出")
    print("-" * 40)

def run_simulation(vehicles, formation, auto_start=True):
    """运行仿真"""
    print(f"🚀 启动仿真: {vehicles}辆车, {formation}编队")
    print("💡 操作提示:")
    print("   - 双击鼠标设置新的目标位置")
    print("   - 使用右侧控制面板调整参数")
    print("   - 点击按钮控制仿真状态")
    print("   - 关闭窗口返回菜单")
    print("-" * 40)
    
    cmd = [
        sys.executable, 
        "formation_simulation_standalone.py",
        "-n", str(vehicles),
        "-f", formation
    ]
    
    if auto_start:
        cmd.append("--auto-start")
    
    try:
        subprocess.run(cmd, check=True)
    except subprocess.CalledProcessError:
        print("❌ 仿真启动失败，请检查依赖是否已安装")
    except KeyboardInterrupt:
        print("⏹️ 用户中断仿真")

def get_user_choice():
    """获取用户选择"""
    while True:
        try:
            choice = input("请输入选项 (0-6): ").strip()
            if choice in ['0', '1', '2', '3', '4', '5', '6']:
                return choice
            else:
                print("❌ 无效选项，请重新输入")
        except KeyboardInterrupt:
            return '0'

def get_custom_params():
    """获取自定义参数"""
    print("\n📝 自定义参数设置:")
    
    while True:
        try:
            vehicles = int(input("车辆数量 (2-20): "))
            if 2 <= vehicles <= 20:
                break
            else:
                print("❌ 车辆数量必须在2-20之间")
        except ValueError:
            print("❌ 请输入有效数字")
    
    print("编队类型选择:")
    print("1. line (直线)")
    print("2. v_shape (V字)")
    print("3. diamond (菱形)")
    print("4. circle (圆形)")
    
    formation_map = {
        '1': 'line',
        '2': 'v_shape',
        '3': 'diamond',
        '4': 'circle'
    }
    
    while True:
        choice = input("请选择编队类型 (1-4): ").strip()
        if choice in formation_map:
            formation = formation_map[choice]
            break
        else:
            print("❌ 无效选项，请重新输入")
    
    auto_start = input("是否自动开始仿真? (y/n): ").strip().lower() == 'y'
    
    return vehicles, formation, auto_start

def check_dependencies():
    """检查依赖"""
    try:
        import numpy
        import matplotlib
        print("✅ 依赖检查通过")
        return True
    except ImportError as e:
        print(f"❌ 缺少依赖: {e}")
        print("请安装: pip install numpy matplotlib")
        return False

def main():
    """主函数"""
    print_banner()
    
    # 检查依赖
    if not check_dependencies():
        return
    
    # 检查仿真文件是否存在
    if not os.path.exists("formation_simulation_standalone.py"):
        print("❌ 找不到仿真文件 formation_simulation_standalone.py")
        return
    
    while True:
        print_menu()
        choice = get_user_choice()
        
        if choice == '0':
            print("👋 感谢使用多车编队仿真系统!")
            break
        elif choice == '1':
            run_simulation(4, 'line')
        elif choice == '2':
            run_simulation(6, 'v_shape')
        elif choice == '3':
            run_simulation(8, 'diamond')
        elif choice == '4':
            run_simulation(6, 'circle')
        elif choice == '5':
            print("🎯 大型编队演示")
            print("这个演示将展示10辆车的复杂编队控制")
            run_simulation(10, 'v_shape')
        elif choice == '6':
            vehicles, formation, auto_start = get_custom_params()
            run_simulation(vehicles, formation, auto_start)
        
        print("\n" + "=" * 40)
        input("按回车键继续...")
        print()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n👋 程序已退出")
    except Exception as e:
        print(f"\n❌ 程序异常: {e}")
        print("请检查环境配置或联系开发者") 