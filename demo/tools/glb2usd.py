import os
import asyncio
import omni.kit.asset_converter

def progress_callback(current_step, total_steps):
    """
    转换进度回调函数，用于在控制台打印进度
    """
    # 为了避免控制台刷屏，只在特定比例时打印进度
    if total_steps > 0 and (current_step % max(1, total_steps // 5) == 0 or current_step == total_steps):
        progress = (current_step / total_steps) * 100
        print(f"   -> 转换进度: {progress:.1f}% ({current_step}/{total_steps})")

async def batch_convert_glb_to_usd(input_dir: str, output_dir: str):
    """
    批量将指定目录下的 GLB 转换为 USD
    """
    # 1. 检查并创建输出目录
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
        print(f"创建输出目录: {output_dir}")

    # 2. 获取所有 glb 文件
    glb_files = [f for f in os.listdir(input_dir) if f.lower().endswith('.glb')]
    
    if not glb_files:
        print(f"[警告] 在 {input_dir} 中没有找到 .glb 文件。")
        return

    print(f"========== 开始批量转换: 共找到 {len(glb_files)} 个资产 ==========")

    # 3. 配置转换参数 (针对自动驾驶仿真优化)
    converter_context = omni.kit.asset_converter.AssetConverterContext()
    
    # 自动驾驶仿真通常使用 Z-up 和 米(Meters)
    converter_context.ignore_up_axis = False 
    converter_context.use_meter_as_world_unit = True 
    converter_context.import_materials = True
    
    # 推荐开启：将纹理转为 DDS 以优化显存占用，这对大规模场景很重要
    # converter_context.texture_format = omni.kit.asset_converter.TextureFormat.DDS 

    task_manager = omni.kit.asset_converter.get_instance()
    success_count = 0
    fail_count = 0

    # 4. 遍历并执行转换
    for glb_file in glb_files:
        input_path = os.path.join(input_dir, glb_file)
        base_name = os.path.splitext(glb_file)[0]
        output_path = os.path.join(output_dir, f"{base_name}.usd")

        print(f"\n[处理中] {glb_file} ...")

        # 创建异步转换任务
        task = task_manager.create_converter_task(
            input_path, 
            output_path, 
            progress_callback, 
            converter_context
        )

        # 等待当前任务完成 (如果你希望极速并发，可以使用 asyncio.gather，但为防显存峰值，建议排队 await)
        success = await task.wait_until_finished()

        if success:
            print(f"   [成功] 已保存至: {output_path}")
            success_count += 1
        else:
            print(f"   [失败] 报错状态: {task.get_status()}")
            fail_count += 1

    print(f"\n========== 转换完成 ==========")
    print(f"总计: {len(glb_files)} | 成功: {success_count} | 失败: {fail_count}")
    print(f"输出路径: {output_dir}")


# ==========================================
# 执行区域 (请修改为你服务器上的实际路径)
# ==========================================
INPUT_DIRECTORY = "/root/liuwei/trellis_glb_assets"  # 替换为你的 GLB 所在目录
OUTPUT_DIRECTORY = "/root/liuwei/isaac_usd_assets"   # 替换为你期望保存 USD 的目录

# 在 Isaac Sim 的 Script Editor 中运行异步函数需要使用 asyncio.ensure_future
asyncio.ensure_future(batch_convert_glb_to_usd(INPUT_DIRECTORY, OUTPUT_DIRECTORY))