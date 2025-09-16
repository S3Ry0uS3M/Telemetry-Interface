#pragma once

#include <QtWidgets>
#include "ui_TyreWidget.h"

class TyreWidget : public QWidget, Ui::TyreForm
{
	Q_OBJECT
public:
	TyreWidget() {
		this->setupUi(this);

	}
	~TyreWidget() {};

public:
	void updatePlot(double xVal)
	{

	}
};