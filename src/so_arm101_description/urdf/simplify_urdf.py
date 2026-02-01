import re
import os

# 这里写你原始 urdf 的文件名
INPUT_FILE = 'so_arm_101.urdf'
# 这里是生成的新文件名
OUTPUT_FILE = 'so_arm_101_simplified.urdf'

def main():
    # 检查文件是否存在
    if not os.path.exists(INPUT_FILE):
        print(f"错误: 找不到文件 {INPUT_FILE}")
        return

    with open(INPUT_FILE, 'r') as f:
        content = f.read()

    print(f"正在读取 {INPUT_FILE} ...")

    # 定义替换逻辑：
    # 找到 <collision> 标签内部，将其中的 <mesh ... /> 替换为 <box ... />
    # 我们保留 origin（位置），只替换 geometry
    
    def replace_mesh_in_collision(match):
        collision_block = match.group(0)
        
        # 如果这个 collision 块里包含 mesh
        if '<mesh' in collision_block:
            # 使用正则表达式把整行 <mesh ... /> 替换掉
            # size="0.05 0.05 0.05" 是一个通用的初始尺寸 (5cm x 5cm x 5cm)
            new_block = re.sub(
                r'<geometry>\s*<mesh.*?\/>\s*<\/geometry>', 
                '<geometry>\n        <box size="0.05 0.05 0.05"/>\n      </geometry>', 
                collision_block, 
                flags=re.DOTALL
            )
            return new_block
        return collision_block

    # 执行全局替换
    # 匹配 <collision> 到 </collision> 之间的所有内容
    new_content = re.sub(
        r'<collision>.*?<\/collision>', 
        replace_mesh_in_collision, 
        content, 
        flags=re.DOTALL
    )

    # 写入新文件
    with open(OUTPUT_FILE, 'w') as f:
        f.write(new_content)

    print(f"成功！已生成简化版文件: {OUTPUT_FILE}")
    print("碰撞模型已全部替换为 5cm 的立方体盒子。")

if __name__ == '__main__':
    main()
