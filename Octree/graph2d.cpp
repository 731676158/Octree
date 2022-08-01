#include "graph2d.h"
#include <algorithm>

using namespace std;

namespace Graph2d {

	graph2d::graph2d()
	{
		width = 700;
		height = 590;
		initgraph(int(width), int(height), SHOWCONSOLE);
		setGrid({ 0,0 }, { 10,10 });
		setlen();
		initAxis();
	}
	graph2d::graph2d(double _width, double _height)
	{
		width = _width;
		height = _height;
		initgraph(int(width), int(height), SHOWCONSOLE);
		setGrid({ 0,0 }, { 10,10 });
		setlen();
		initAxis();
	}
	graph2d::graph2d(double _width, double _height, Point _pointlb, Point _pointrt)
	{
		width = _width;
		height = _height;
		initgraph(int(width), int(height), SHOWCONSOLE);
		setGrid(_pointlb, _pointrt);
		setlen();
		initAxis();
	}
	graph2d::~graph2d()
	{
		closegraph();
	}
	wchar_t* graph2d::ctow(const char* str)
	{
		int length = strlen(str) + 1;
		wchar_t* t = (wchar_t*)malloc(sizeof(wchar_t) * length);
		memset(t, 0, length * sizeof(wchar_t));
		MultiByteToWideChar(CP_ACP, 0, str, strlen(str), t, length);
		return t;
	}
	void graph2d::setlen(int _len)
	{
		x_len = max(int(1 + log10(max(pointlb.x, pointrt.x))), _len);
		y_len = max(int(1 + log10(max(pointlb.y, pointrt.y))), _len);
	}
	std::string graph2d::dtos(double _num, char _axis)
	{
		int _len = 3;
		std::string _str = "";	// 储存结果
		int _lg = 0;			// 10^__lg
		if (_axis == 'x' || _axis == 'X')
			_len = x_len;
		if (_axis == 'y' || _axis == 'Y')
			_len = y_len;
		if (int(_num * pow(10.0, _len - 1)) == 0) {
			for (int i = 0; i < _len - 1; i++) {
				_str += '0';
			}
			return "0." + _str;
		}
		if (_num < 0) { _num = -_num; _str += '-'; }
		else { _str += ' '; }
		while (int(_num) > 1) { _num /= 10; _lg++; }
		while (int(_num) < 1 && _lg > 0) { _num *= 10; _lg--; }
		for (int i = 0; i < _len; i++) {
			_str += ('0' + int(_num) % 10);
			if (i == _lg && i + 1 != _len) {
				_str += '.';
			}
			_num *= 10;
		}
		return _str;
	}
	void graph2d::waitKey(int _delay)
	{
		_getch();
	}
	void graph2d::setGrid(Point _pointlb, Point _pointrt)
	{
		pointlb = { min(_pointlb.x,_pointrt.x), min(_pointlb.y,_pointrt.y) };
		pointrt = { max(_pointlb.x,_pointrt.x), max(_pointlb.y,_pointrt.y) };
	}
	void graph2d::drawGrid()
	{
		setlinecolor(0xE0E0E0);
		setlinestyle(PS_SOLID, 1);
		for (int i = 1; i < 10; i++) {
			line(numConversion(0.078 * width * i + 0.12 * width, 'x'), numConversion(0.1 * height, 'y'), numConversion(0.078 * width * i + 0.12 * width, 'x'), numConversion(0.88 * height, 'y'));
			line(numConversion(0.12 * width, 'x'), numConversion(0.078 * height * i + 0.1 * height, 'y'), numConversion(0.9 * width, 'x'), numConversion(0.078 * height * i + 0.1 * height, 'y'));
		}
	}
	void graph2d::drawAxisX()
	{
		std::string _str = "     ";
		for (int i = 0; i <= 10; i++) {
			_str += dtos((pointrt.x - pointlb.x) / 10 * i + pointlb.x, 'x') + "  ";
		}
		RECT r = { numConversion(0,'x'), numConversion(0.04 * height,'y'), numConversion(width,'x'), numConversion(0.11 * height,'y') };
		settextcolor(BLACK);
		settextstyle(int((24 - 2 * x_len) * (width / 700) * 0.9), 0, _T("宋体"));
		drawtext(ctow(_str.c_str()), &r, DT_CENTER | DT_VCENTER | DT_SINGLELINE);
	}
	void graph2d::drawAxisY()
	{
		std::string _str = "\n\n\n\n\n";
		for (int i = 10; i >= 0; i--) {
			_str += dtos((pointrt.y - pointlb.y) / 10 * i + pointlb.y, 'y') + "\n\n\n";
		}
		RECT r = { numConversion(0,'x'), numConversion(height,'y'), numConversion(0.11 * width,'x'), numConversion(0,'y') };
		settextcolor(BLACK);
		settextstyle(int(15 * height / 590), 0, _T("宋体"));
		drawtext(ctow(_str.c_str()), &r, DT_NOCLIP | DT_RIGHT);
	}
	bool graph2d::isBorder(Point _point)
	{
		return (pointlb.x <= _point.x && _point.x <= pointrt.x && pointlb.y <= _point.y && _point.y <= pointrt.y);
	}
	int graph2d::numConversion(double _num, char _axis)
	{
		if (_axis == 'x' || _axis == 'X') {
			return int(_num);
		}
		if (_axis == 'y' || _axis == 'Y') {
			return int(height - _num);
		}
		return 0;
	}
	Point graph2d::fucCSDataToAbsCSData(Point _point)
	{
		return { ((_point.x - pointlb.x) / (pointrt.x - pointlb.x) * 0.78 * width + 0.12 * width),((_point.y - pointlb.y) / (pointrt.y - pointlb.y) * 0.78 * height + 0.1 * height) };
	}
	Point graph2d::absCSDataToFucCSData(Point _point)
	{
		return { ((_point.x - 0.12 * width) * (pointrt.x - pointlb.x) / 0.78 / width + pointlb.x),((_point.y - 0.1 * height) * (pointrt.y - pointlb.y) / 0.78 / height + pointlb.y) };
	}
	void graph2d::showError(std::string _err)
	{
		std::cout << "Maybe a " << _err << " is not in the coordinates" << std::endl;
	}
	void graph2d::setBackgroundColor(COLORREF _color)
	{
		setbkcolor(_color);
		cleardevice();
	}
	void graph2d::setAxisColor()
	{
		drawRectangle({ 0.12 * width ,0.1 * height }, { 0.9 * width ,0.88 * height }, BLACK, WHITE);
	}
	void graph2d::drawRectangle(Point _pointlb, Point _pointrt, COLORREF _colorl, COLORREF _colorf, int _style)
	{
		setlinecolor(_colorl);
		setfillcolor(_colorf);
		setfillstyle(_style);
		fillrectangle(numConversion(_pointlb.x, 'x'), numConversion(_pointrt.y, 'y'), numConversion(_pointrt.x, 'x'), numConversion(_pointlb.y, 'y'));
	}
	void graph2d::initAxis()
	{
		setBackgroundColor();
		setAxisColor();
		drawGrid();
		drawAxisX();
		drawAxisY();
	}
	void graph2d::plot(Point _point, COLORREF _color, int _size, int _type)
	{
		if (isBorder(_point)) {
			setlinecolor(_color);
			setfillcolor(_color);
			setfillstyle(_type);
			fillcircle(numConversion(fucCSDataToAbsCSData(_point).x, 'x'), numConversion(fucCSDataToAbsCSData(_point).y, 'y'), _size);
		}
		else {
			showError("point");
		}
	}
	void graph2d::plot(std::vector<Point> _point, COLORREF _color, int _thickness, int _type)
	{
		setlinecolor(_color);
		setlinestyle(_type, _thickness);
		for (int i = 1; i < _point.size(); i++) {
			if (isBorder(_point[i - 1]) && isBorder(_point[i])) {
				line(numConversion(fucCSDataToAbsCSData(_point[i - 1]).x, 'x'), numConversion(fucCSDataToAbsCSData(_point[i - 1]).y, 'y'), numConversion(fucCSDataToAbsCSData(_point[i]).x, 'x'), numConversion(fucCSDataToAbsCSData(_point[i]).y, 'y'));
			}
			else {
				showError("line");
			}
		}
	}
	void graph2d::plot(double _start, double _end, double (*_f)(double), double _step, COLORREF _color, int _thickness, int _type)
	{
		std::vector<Point> _point;
		for (double i = _start; i <= _end; i += _step) {
			_point.push_back({ i, _f(i) });
		}
		plot(_point, _color, _thickness, _type);
	}
	void graph2d::title(std::string _str)
	{
		RECT r = { numConversion(0,'x'), numConversion(0.88 * height,'y'), numConversion(width,'x'), numConversion(height,'y') };
		settextcolor(BLACK);
		settextstyle(int(20 * height / 590), 0, _T("宋体"));
		drawtext(ctow(_str.c_str()), &r, DT_CENTER | DT_VCENTER | DT_SINGLELINE);
	}
	void graph2d::xlabel(std::string _str)
	{
		RECT r = { numConversion(0,'x'), numConversion(0,'y'), numConversion(width,'x'), numConversion(0.05 * height,'y') };
		settextcolor(BLACK);
		settextstyle(int(20 * height / 590), 0, _T("宋体"));
		drawtext(ctow(_str.c_str()), &r, DT_CENTER | DT_VCENTER | DT_SINGLELINE);
	}
	void graph2d::ylabel(std::string _str)
	{
		RECT r = { numConversion(0.01 * width,'x'), numConversion(0.6 * height,'y'), numConversion(0.05 * width,'x'), numConversion(0,'y') };
		settextcolor(BLACK);
		settextstyle(int(20 * width / 700), 0, _T("宋体"));
		drawtext(ctow(_str.c_str()), &r, DT_NOCLIP | DT_LEFT | DT_WORDBREAK);
	}
	void graph2d::text(std::string _str)
	{
		//TODO
	}
	void graph2d::legend(std::string _str, COLORREF _color, int _num)
	{
		//TODO
	}
	void graph2d::mouseClick()
	{
		//TODO
	}

}