使用方法:
1. 测试单个用例，假定用例ID为DEMO_CASE_001:
./H1_SDK_TEST -case "DEMO_CASE_001"

2. 测试用例序列，序列在文件中指定，指定方法为一行一个用例ID，假定当前目录存在smoking.id，文件内容为:
DEMO_CASE_001
DEMO_CASE_002
DEMO_CASE_003
./H1_SDK_TEST -casefile ./smoking.id

3. 强制重新测试用例，添加-force参数:
./H1_SDK_TEST -force -case "DEMO_CASE_001"
./H1_SDK_TEST -force -casefile ./smoking.id

4. 测试前询问用户是否测试该用例:
./H1_SDK_TEST -confirm -case "DEMO_CASE_001"
./H1_SDK_TEST -force -confirm -casefile ./smoking.id

5. 指定测试报告文件，默认为"./report.cr",可通过-report重新指定:
./H1_SDK_TEST -report ./H1-v2.9.20220630.tr -casefile ./smoking.id

6.测试时指定型号：
./DianaApi_C -case "DEMO_CASE_001" -Thor5

7.使用fastddsgen工具编译IDL文件生成需要的数据源文件
fastddsgen HelloWorld.idl



H20测试用例执行指令：
./H1_SDK_TEST -force -case "H20_FC_LeftLeg_001" -H20


