#pragma once

#include <QtWidgets>
#include "ui_PlotWidget.h"

class PlotWidget : public QWidget, Ui::PlotForm
{
	Q_OBJECT
public:
	PlotWidget() {
		this->setupUi(this);

		plot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
		plot->addGraph();
		plot->graph(0)->setPen(QPen(Qt::blue));
		plot->graph(0)->setName("");

		plot->legend->setVisible(true);
		plot->legend->setBrush(Qt::NoBrush); // Fully transparent background
		plot->legend->setBorderPen(Qt::NoPen);

		this->selectedChannel.append({ this->horizontalLayout, std::function<double()>{} });

		connect(this->comboBox, static_cast<void(QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, [this]() {
			updateSelectedChannel(this->comboBox, 0);
		});
		connect(this->addChannelButton, &QPushButton::clicked, this, &PlotWidget::addChannelBox);
		connect(this->deleteChannelButton, &QPushButton::clicked, this, &PlotWidget::deleteChannelBox);
	}
	~PlotWidget() {};

public:
	void setSelectableChannels(QStringList list)
	{
		this->comboBox->addItems(list);
		this->comboBox->setCurrentIndex(-1); // blank selection
	}

	void setAvailableChannels(std::map<QString, std::function<double()>>* map)
	{
		this->availableChannels = map;
	}

	void updatePlot(double xVal)
	{
		for (int i = 0; i < this->selectedChannel.size(); i++)
		{
			if (!get<1>(this->selectedChannel[i]))
				return;

			plot->graph(i)->addData(xVal, get<1>(this->selectedChannel[i])());
			plot->graph(i)->setName(QString("%1").arg(get<1>(this->selectedChannel[i])(), 0, 'f', 2));
		}
		plot->xAxis->setRange(xVal - 20, xVal);
		// Get the currently visible x range
		QCPRange visibleX = plot->xAxis->range();

		double globalMinY = std::numeric_limits<double>::max();
		double globalMaxY = -std::numeric_limits<double>::max();

		for (int i = 0; i < plot->graphCount(); ++i)
		{
			QCPGraph* graph = plot->graph(i);
			if (!graph->data()->isEmpty())
			{
				// Get the subset of points within the visible x range
				QCPDataContainer<QCPGraphData>::const_iterator itBegin = graph->data()->findBegin(visibleX.lower);
				QCPDataContainer<QCPGraphData>::const_iterator itEnd = graph->data()->findEnd(visibleX.upper);

				for (auto it = itBegin; it != itEnd; ++it)
				{
					if (it->value < globalMinY) globalMinY = it->value;
					if (it->value > globalMaxY) globalMaxY = it->value;
				}
			}
		}

		// Add 10% margin only if we found valid data
		if (globalMinY < globalMaxY)
		{
			double margin = (globalMaxY - globalMinY) * 0.1;
			plot->yAxis->setRange(globalMinY - margin, globalMaxY + margin);
		}
		plot->replot();
	}

protected:
	QList<std::tuple<QHBoxLayout*, std::function<double()>>> selectedChannel;
	std::map<QString, std::function<double()>>* availableChannels = {};

private:
	void updateSelectedChannel(QComboBox* combo, int index)
	{
		if (availableChannels == nullptr)
			return;
		get<1>(this->selectedChannel[index]) = availableChannels->at(combo->currentText());
	}

private slots:
	void addChannelBox()
	{
		QVBoxLayout* mainLayout = qobject_cast<QVBoxLayout*>(this->groupBox->layout());
		if (!mainLayout)
			return;

		QHBoxLayout* newHBox = new QHBoxLayout();
		QLabel* label = new QLabel();
		label->setText("Channel:");
		QSizePolicy sizePolicy(QSizePolicy::Maximum, QSizePolicy::Preferred);
		sizePolicy.setHorizontalStretch(0);
		sizePolicy.setVerticalStretch(0);
		sizePolicy.setHeightForWidth(label->sizePolicy().hasHeightForWidth());
		label->setSizePolicy(sizePolicy);

		QComboBox* combo = new QComboBox();
		combo->setMinimumWidth(150);
		combo->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
		QStringList list;
		for (int i = 0; i < this->comboBox->count(); i++)
			list.append(this->comboBox->itemText(i));
		combo->addItems(list);
		combo->setCurrentIndex(-1); // blank selection

		newHBox->addWidget(label);
		newHBox->addWidget(combo);

		// Insert above the "+" button
		int insertIndex = mainLayout->count() - 2; // last is "+"
		mainLayout->insertLayout(insertIndex, newHBox);

		int index = this->selectedChannel.size();
		this->selectedChannel.append({ newHBox, std::function<double()>{} });

		// Add graph
		int hue = QRandomGenerator::global()->bounded(360); // hue: 0–359
		QColor color;
		color.setHsv(hue, 255, 200); // (hue, saturation, value)

		QPen pen(color);
		this->plot->addGraph();
		this->plot->graph(plot->graphCount() - 1)->setPen(QPen(color));
		this->plot->graph(plot->graphCount() - 1)->setName("");

		connect(combo, static_cast<void(QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, [this, combo, index]() {
			updateSelectedChannel(combo, index);
		});
	}

	void deleteChannelBox()
	{
		if (selectedChannel.size() <= 1)
			return;

		// Remove last channel
		QVBoxLayout* mainLayout = qobject_cast<QVBoxLayout*>(this->groupBox->layout());
		if (!mainLayout)
			return;

		auto index = selectedChannel.size() - 1;
		QHBoxLayout* hLayout = std::get<0>(selectedChannel[index]);
		QComboBox* combo = hLayout->findChild<QComboBox*>();
		clearLayout(hLayout);

		if (hLayout)
		{
			mainLayout->removeItem(hLayout);
			hLayout->deleteLater();            // deletes all child widgets/layouts
		}
		selectedChannel.removeLast();

		// Remove graph
		this->plot->removeGraph(this->plot->graphCount() - 1);
	}

	void clearLayout(QLayout* layout)
	{
		if (!layout) return;

		QLayoutItem* item;
		while ((item = layout->takeAt(0)) != nullptr)  // take items one by one
		{
			if (QWidget* widget = item->widget()) {
				widget->setParent(nullptr); // detach from layout
				widget->deleteLater();      // delete widget
			}
			if (QLayout* childLayout = item->layout()) {
				clearLayout(childLayout);   // recursively clear child layouts
			}
			delete item; // delete the layout item itself
		}
	}
};
