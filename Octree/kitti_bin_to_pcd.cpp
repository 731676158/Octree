#include <iostream>
using namespace std;

int main()
{
	int32_t num = 1000000;
	float* data = (float*)malloc(num * sizeof(float));
	// 点
	float* px = data + 0;
	float* py = data + 1;
	float* pz = data + 2;
	float* pr = data + 3;//反射强度
	// 读取点云数据
	FILE* stream;
	fopen_s(&stream, "data\\0000000000.bin", "rb");
	num = fread(data, sizeof(float), num, stream) / 4;//读入点云数据，大概10万+个点
	fclose(stream);
	/**转换成PCD**/
	//写文件声明
	FILE* writePCDStream;
	fopen_s(&writePCDStream, "0000000000.pcd", "wb");
	fprintf(writePCDStream, "VERSION 0.7\n");//版本说明
	fprintf(writePCDStream, "FIELDS x y z\n");//维度说明
	fprintf(writePCDStream, "SIZE 4 4 4\n");//占用字节说明
	fprintf(writePCDStream, "TYPE F F F\n");//具体数据类型定义
	fprintf(writePCDStream, "WIDTH %d\n", num);//点数量
	fprintf(writePCDStream, "HEIGHT 1\n");//无序点云默认为1
	fprintf(writePCDStream, "POINTS %d\n", num);//点数量
	fprintf(writePCDStream, "DATA ascii\n");//文档使用字符类型shuom
	//写点云数据
	for (int32_t i = 0; i < num; i++)
	{
		fprintf(writePCDStream, "%f %f %f\n", *px, *py, *pz);
		px += 4; py += 4; pz += 4; pr += 4;
	}
	fclose(writePCDStream);

	return 0;
}