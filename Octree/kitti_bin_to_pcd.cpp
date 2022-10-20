#include <iostream>
using namespace std;

int main()
{
	int32_t num = 1000000;
	float* data = (float*)malloc(num * sizeof(float));
	// ��
	float* px = data + 0;
	float* py = data + 1;
	float* pz = data + 2;
	float* pr = data + 3;//����ǿ��
	// ��ȡ��������
	FILE* stream;
	fopen_s(&stream, "data\\0000000000.bin", "rb");
	num = fread(data, sizeof(float), num, stream) / 4;//����������ݣ����10��+����
	fclose(stream);
	/**ת����PCD**/
	//д�ļ�����
	FILE* writePCDStream;
	fopen_s(&writePCDStream, "0000000000.pcd", "wb");
	fprintf(writePCDStream, "VERSION 0.7\n");//�汾˵��
	fprintf(writePCDStream, "FIELDS x y z\n");//ά��˵��
	fprintf(writePCDStream, "SIZE 4 4 4\n");//ռ���ֽ�˵��
	fprintf(writePCDStream, "TYPE F F F\n");//�����������Ͷ���
	fprintf(writePCDStream, "WIDTH %d\n", num);//������
	fprintf(writePCDStream, "HEIGHT 1\n");//�������Ĭ��Ϊ1
	fprintf(writePCDStream, "POINTS %d\n", num);//������
	fprintf(writePCDStream, "DATA ascii\n");//�ĵ�ʹ���ַ�����shuom
	//д��������
	for (int32_t i = 0; i < num; i++)
	{
		fprintf(writePCDStream, "%f %f %f\n", *px, *py, *pz);
		px += 4; py += 4; pz += 4; pr += 4;
	}
	fclose(writePCDStream);

	return 0;
}