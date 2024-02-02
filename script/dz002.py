import os
Import("env")
# print(env.Dump())
try:
    PIOENV = env.get("PIOENV")
    PROJECT_DIR = env.get("PROJECT_DIR")
    outDir = os.path.abspath(PROJECT_DIR+"/data")
    os.chdir("../ts")
    print("当前：", os.getcwd())
    cmd=f"esno src/dz002-cppWeb/writeFile.ts --outDir={outDir}"
    env.Execute(cmd)
except Exception as e:
    print("错误", e)
os.chdir(PROJECT_DIR)  # 返回原始工作目录
print("当前：", os.getcwd())
# try:
#     mcuPath = env.get("PROJECT_DIR")
#     srcFilePath = os.path.abspath(mcuPath+"/../d6227-mcu-ts/src/useStore.ts")
#     saveFilePath = os.path.abspath(mcuPath+"/data/config.json")
#     if os.path.exists(saveFilePath):
#         os.remove(saveFilePath)
#     print('srcFilePath:', srcFilePath, ";\nsaveFilePath:", saveFilePath)
#     with open(srcFilePath, 'r') as f:
#         ts_content = f.read()
#     startstr = "const mcu00: mcu00_t = "
#     start_index = ts_content.index(startstr) + len(startstr)
#     end_index = ts_content.find("};", start_index)+1
#     mcu00_str = ts_content[start_index:end_index].strip()
#     print(mcu00_str)
#     if (mcu00_str):
#         mcu00 = eval(mcu00_str)
#         with open(saveFilePath, 'w') as f:
#             json.dump(mcu00, f)
#     else:
#         print("\nmcu00 undefind\n")
#     def uploadfsfiles_to_target(env,*args, **kwargs):
#         # firmware_path = env.get("PROG_PATH")
#         # print("***************************")
#         # print(firmware_path)
#         # print("*************************")
#         result = subprocess.run(["platformio", "run", "--target", "uploadfs", "--environment", "mcu00", "--upload-port", "COM3"])
#         print(result)
#     env.AddPreAction("upload", uploadfsfiles_to_target)
# except subprocess.CalledProcessError as e:
#     print(e)
# finally:
#     print("\nsuccess\n")

# try:
#     # result = subprocess.run(['where', 'platformio'],
#     #                         capture_output=True, text=True)
#     # output_lines = result.stdout.splitlines()
#     # runtarget = output_lines[0]+" run --target "
#     mcuPath = env.get("PROJECT_DIR")
#     srcFilePath = os.path.abspath(mcuPath+"/../d6227-mcu-ts/src/useStore.ts")
#     saveFilePath = os.path.abspath(mcuPath+"/data/config.json")
#     if os.path.exists(saveFilePath):
#         os.remove(saveFilePath)
#     print('srcFilePath:', srcFilePath, ";\nsaveFilePath:", saveFilePath)
#     # 构建命令字符串，使用console.log输出mcuConfig的值
#     # process.chdir(\'../d6227-mcu-ts\');
#     # os.chdir("../d6227-mcu-ts")
#     # output = subprocess.check_output("ts-node ./src/useConfig.ts",encoding='utf-8')
#     # print(output)
#     # subprocess.check_output(["npx", "ts-node", "-e", f"require('fs').writeFileSync('{saveFilePath}', require('{srcFilePath}').mcuConfig)"])
#     with open(srcFilePath, 'r') as f:
#         ts_content = f.read()
#     startstr = "const mcu00: mcu00_t = "
#     start_index = ts_content.index(startstr) + len(startstr)
#     end_index = ts_content.find("};", start_index)+1
#     mcu00_str = ts_content[start_index:end_index].strip()
#     print(mcu00_str)
#     if (mcu00_str):
#         mcu00 = eval(mcu00_str)
#         with open(saveFilePath, 'w') as f:
#             json.dump(mcu00, f)
#         # print(runtarget)
#         # os.system('"' + runtarget + 'buildfs"')
#         # os.system('"'+runtarget+'size"')
#         # os.system('"' + runtarget + 'uploadfs"')
#     else:
#         print("\nmcu00 undefind\n")
# except subprocess.CalledProcessError as e:
#     print(e)
# finally:
#     print("\npy success\n")


# if (len(BUILD_TARGETS) == 0 or "upload" in BUILD_TARGETS):
#     try:
#         mcuPath = env.get("PROJECT_DIR")
#         mcudataPath = os.path.abspath(mcuPath+"/data")
#         uiPath = os.path.abspath(mcuPath+"/../d6227-mcu-ts")
#         print("当前工作目录 : " + mcuPath)
#         os.chdir(uiPath)
#         print("当前工作目录 : " + os.getcwd())
#         env.Execute("pnpm run build --mode app-spiffs --outDir "+mcudataPath)
#     except Exception as e:
#         print(e)
#     finally:
#         os.chdir(mcuPath)
#         print("当前工作目录 : " + mcuPath)
#     try:
#         p = "C:\\Users\\13520\\.platformio\\penv\\Scripts\\platformio.exe run --target"
#         os.system('"' + p + ' buildfs"')
#         os.system('"'+p+' size"')
#         os.system('"' + p + ' uploadfs"')
#     except Exception as e:
#         print(e + 'uploadFs error')
#     finally:
#         print("uploadfs success")

# if (len(BUILD_TARGETS) == 0 or "upload" in BUILD_TARGETS):
#     try:
#         # mcuPath = env.get("PROJECT_DIR")
#         # srcFilePath=os.path.abspath(mcuPath+"/../d6227-mcu-ts/src/config.json")
#         # saveFilePath=os.path.abspath(mcuPath+"/data/config.json")
#         # if os.path.exists(saveFilePath):
#         #     os.remove(saveFilePath)
#         # msg=shutil.copyfile(srcFilePath,saveFilePath)
#         result = subprocess.run(['where', 'platformio'], capture_output=True, text=True)
#         output_lines = result.stdout.splitlines()
#         p= output_lines[0]+" run --target "
#         # os.system('"' + p + 'buildfs" && "' + p + 'size" && "' + p + 'uploadfs"')
#         os.system('"' + p + 'buildfs"')
#         os.system('"'+p+'size"')
#         os.system('"' + p + 'uploadfs"')
#     except Exception as e:
#         print(e)
# finally:
#     print("\n\n\nuploadfs success;srcFilePath=%s;saveFilePath=%s;copy msg=%s;exe path=%s\n\n\n" % (srcFilePath, saveFilePath,msg,p))
