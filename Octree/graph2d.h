#pragma once
#include <iostream>
#include <graphics.h>
#include <conio.h>
#include <vector>
#include <math.h>
#define f(_lambda) ([](double x) {return (_lambda); })

namespace Graph2d {
	typedef struct mypoint {
		double x;
		double y;
	}Point;
	class graph2d
	{
	private:
		double height;											// 画板高度
		double width;											// 画板宽度
		Point pointlb;											// 坐标轴左下角的点
		Point pointrt;											// 坐标轴右上角的点
		int x_len;												// x轴字的宽度
		int y_len;												// y轴字的宽度
	public:
		/*********************************
		 *
		 *	初始化、坐标转换、绘制坐标等等
		 *
		 *********************************/
		graph2d();												// 初始化为默认值
		graph2d(double _width, double _height);					// 初始化画板宽高
		graph2d(double _width, double _height, Point _pointlb, Point _pointrt);		// 初始化画板宽高及网格
		~graph2d();												// 析构函数
		void waitKey(int _delay = 0);							// 等待关闭
	private:
		/*********************************
		 *
		 *	坐标转换、绘制坐标等等
		 *
		 *********************************/
		wchar_t* ctow(const char* str);							// char* to wchar_t*
		void setlen(int _len = 3);								// 设置坐标文本长度
		std::string dtos(double _num, char _axis);				// string to double
		void setGrid(Point _pointlb, Point _pointrt);			// 设置网格
		void drawGrid();										// 绘制网格
		void drawAxisX();										// 绘制X轴
		void drawAxisY();										// 绘制Y轴
		bool isBorder(Point _point);							// 是否在边界内
		int numConversion(double _num, char _axis);				// 将y轴颠倒
		Point fucCSDataToAbsCSData(Point _point);				// 方程的点转换到画板的点
		Point absCSDataToFucCSData(Point _point);				// 画板的点转换到方程的点
		void showError(std::string _err);						// 显示错误
		void drawRectangle(Point _pointlb, Point _pointrt, COLORREF _colorl, COLORREF _colorf, int _style = BS_SOLID);	//画正方形
		void setBackgroundColor(COLORREF _color = 0xEAEAEA);	// 设置画板背景颜色
		void setAxisColor();									// 设置坐标系背景颜色
		void initAxis();										// 初始化坐标轴内的信息
		void mouseClick();										// 鼠标点击来获取该点函数坐标
	public:
		/*********************************
		 *
		 *	绘制坐标方程函数
		 *
		 *********************************/
		void plot(Point _point, COLORREF _color = RED, int _size = 3, int _type = BS_SOLID);		// 绘制点
		void plot(std::vector<Point> _point, COLORREF _color = BLACK, int _thickness = 3, int _type = PS_SOLID);		// 绘制一连串的线
		void plot(double _start, double _end, double (*_f)(double), double _step = 0.1, COLORREF _color = BLACK, int _thickness = 3, int _type = PS_SOLID);		// 绘制f(x)方程
		void title(std::string _str);							// 标题
		void xlabel(std::string _str);							// x轴文本注释
		void ylabel(std::string _str);							// y轴文本注释,可以用空格换行
		void text(std::string _str);							// 在坐标中添加注释
		void legend(std::string _str, COLORREF _color, int _num);// 标签
	};


}