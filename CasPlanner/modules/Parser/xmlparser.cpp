/***************************************************************************
 *   Copyright (C) 2006 - 2007 by                                          *
 *      Tarek Taha, CAS-UTS  <tataha@tarektaha.com>                        *
 *                                                                         *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Steet, Fifth Floor, Boston, MA  02111-1307, USA.          *
 ***************************************************************************/
#include "xmlparser.h"

XmlParser::XmlParser()
{
	
}

XmlParser::XmlParser(QFile *file)
{
	this->configFile = file;
}

XmlParser::XmlParser(QString fileName)
{
	this->configFileName = fileName;
}

void XmlParser::read()
{
    QFile file(configFileName);
    if (!file.open(QIODevice::ReadOnly))
        return;
    if (!configFileDom.setContent(&file)) 
    {
        file.close();
        return;
    }
    file.close();

    // print out the element names of all elements that are direct children
    // of the outermost element.
    QDomElement docElem = configFileDom.documentElement();
    QDomNode n = docElem.firstChild();
    while(!n.isNull()) 
    {
        QDomElement e = n.toElement(); // try to convert the node to an element.
        if(!e.isNull()) 
        {
            std::cout << qPrintable(e.tagName())<<"\n"; // the node really is an element.
            QDomNamedNodeMap attr = e.attributes();
            std::cout<<"Attr Size:="<<attr.size();
            for(int i=0;i<attr.size();i++)
            {
            	QDomAttr attrNode = attr.item(i).toAttr();
            	std::cout << qPrintable(attrNode.value())<<"\n";
            }
        }
        n = n.nextSibling();
    }
}

XmlParser::~XmlParser()
{
	
}
