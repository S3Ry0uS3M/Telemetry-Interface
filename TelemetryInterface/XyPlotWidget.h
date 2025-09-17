#pragma once

#include <QtWidgets>
#include "ui_XyPlotWidget.h"

enum eAxisType
{
	xAxis,
	yAxis
};

class XyPlotWidget : public QWidget, Ui::XyPlotForm
{
	Q_OBJECT
public:
	XyPlotWidget() {
		this->setupUi(this);

		/*plot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
		plot->addGraph();
		plot->graph(0)->setPen(QPen(Qt::blue));
		plot->graph(0)->setName("");*/

		curve = new QCPCurve(plot->xAxis, plot->yAxis);
		curve->setPen(QPen(Qt::blue));
		curve->setName("");
		plot->legend->setVisible(true);
		plot->legend->setBrush(Qt::NoBrush); // Fully transparent background
		plot->legend->setBorderPen(Qt::NoPen);

		this->selectedChannels = { std::function<double()>{}, std::function<double()>{} };

		connect(this->xComboBox, static_cast<void(QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, [this]() {
			updateSelectedChannel(this->xComboBox, eAxisType::xAxis);
		});
		connect(this->yComboBox, static_cast<void(QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, [this]() {
			updateSelectedChannel(this->yComboBox, eAxisType::yAxis);
		});
	}
	~XyPlotWidget() {};

protected:
	std::tuple<std::function<double()>, std::function<double()>> selectedChannels; // (xFunc, yFunc)
	std::map<QString, std::function<double()>>* availableChannels = {};
	QCPCurve* curve = nullptr;

public:
	void setSelectableChannels(QStringList list)
	{
		this->xComboBox->clear();
		this->xComboBox->addItems(list);
		this->xComboBox->setCurrentIndex(-1); // blank selection

		this->yComboBox->clear();
		this->yComboBox->addItems(list);
		this->yComboBox->setCurrentIndex(-1); // blank selection
	}

	void setAvailableChannels(std::map<QString, std::function<double()>>* map)
	{
		this->availableChannels = map;
	}
	
	void updatePlot()
	{
		if (!get<0>(this->selectedChannels) || !get<1>(this->selectedChannels))
			return;

		double x = std::get<0>(this->selectedChannels)();
		double y = std::get<1>(this->selectedChannels)();

		curve->addData(x, y);
		curve->setName(QString("x: %1 | y: %2").arg(x, 0, 'f', 2).arg(y, 0, 'f', 2));
		plot->xAxis->setRange(x + this->xRangeMin->value(), x + this->xRangeMax->value());
		plot->yAxis->setRange(y + this->yRangeMin->value(), y + this->yRangeMax->value());
		plot->replot();
	}

private:
	void updateSelectedChannel(QComboBox* combo, eAxisType axis)
	{
		if (availableChannels == nullptr)
			return;

		int comboIndex = combo->currentIndex();
		if (comboIndex < 0)
			return; // invalid index

		QString key = combo->currentText();
		auto it = availableChannels->find(key);
		if (it != availableChannels->end()) {
			if (axis == eAxisType::xAxis)
				std::get<0>(this->selectedChannels) = it->second;
			else
				std::get<1>(this->selectedChannels) = it->second;
		}
	}
};