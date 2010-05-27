/***************************************************************************
 *   Vision Classification Library                                         *
 *   Copyright (C) 2010 by:                                                *
 *      Tarek Taha, CAS-UTS  <tataha@cas.edu.au>                           *
 *      Dan Maynes-Aminzade  <monzy@stanford.edu>                          *
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
 *   51 Franklin Steet, Fifth Floor, Boston, MA  02110-1301, USA.          *
 ***************************************************************************/
#include "OneDollar.h"

// number of default templates included at startup
#define NumTemps 0
// number of points used in resampled paths
#define NumPoints 64

#define SquareSize 250.0
#define HalfDiagonal (0.5 * sqrt(250.0 * 250.0 + 250.0 * 250.0))
#define AngleRange 45.0
#define AnglePrecision 2.0
#define PHI (0.5 * (-1.0 + sqrt(5.0)))

Template::Template(string name, vector<OneDollarPoint> points)
{
    m_name = name;
    m_points = Resample(points, NumPoints);
    m_points = RotateToZero(m_points);
    m_points = ScaleToSquare(m_points, GESTURE_SQUARE_SIZE);
    m_points = TranslateToOrigin(m_points);
}

Template::Template(FILE *src)
{
    // first read the name from the datafile
    char name[MAX_PATH];
    fread(name,sizeof(char),MAX_PATH,src);
    m_name = name;

    // now read the number of points
    int nPoints = 0;
    fread(&nPoints, sizeof(int), 1, src);

    // read in each point
    double x, y;
    for (int i=0; i<nPoints; i++)
    {
        fread(&x, sizeof(double), 1, src);
        fread(&y, sizeof(double), 1, src);
        m_points.push_back(OneDollarPoint(x,y));
    }
}

void Template::WriteToFile(FILE *dst)
{
    // first write the name to the datafile
    char name[MAX_PATH];
    strcpy(name, m_name.c_str());
    fwrite(name, sizeof(char), MAX_PATH, dst);

    // now write the number of points
    int nPoints = m_points.size();
    fwrite(&nPoints, sizeof(int), 1, dst);

    // write out each point
    double x, y;
    for (int i=0; i<nPoints; i++)
    {
        x = m_points[i].m_x;
        y = m_points[i].m_y;
        fwrite(&x, sizeof(double), 1, dst);
        fwrite(&y, sizeof(double), 1, dst);
    }
}

int Template::GetLength()
{
    return m_points.size();
}

Recognizer::Recognizer()
{
    /*
	// Initialize with default templates
	OneDollarPoint trianglePts[66] = { OneDollarPoint(137,139),OneDollarPoint(135,141),OneDollarPoint(133,144),OneDollarPoint(132,146),OneDollarPoint(130,149),OneDollarPoint(128,151),OneDollarPoint(126,155),OneDollarPoint(123,160),OneDollarPoint(120,166),OneDollarPoint(116,171),OneDollarPoint(112,177),OneDollarPoint(107,183),OneDollarPoint(102,188),OneDollarPoint(100,191),OneDollarPoint(95,195),OneDollarPoint(90,199),OneDollarPoint(86,203),OneDollarPoint(82,206),OneDollarPoint(80,209),OneDollarPoint(75,213),OneDollarPoint(73,213),OneDollarPoint(70,216),OneDollarPoint(67,219),OneDollarPoint(64,221),OneDollarPoint(61,223),OneDollarPoint(60,225),OneDollarPoint(62,226),OneDollarPoint(65,225),OneDollarPoint(67,226),OneDollarPoint(74,226),OneDollarPoint(77,227),OneDollarPoint(85,229),OneDollarPoint(91,230),OneDollarPoint(99,231),OneDollarPoint(108,232),OneDollarPoint(116,233),OneDollarPoint(125,233),OneDollarPoint(134,234),OneDollarPoint(145,233),OneDollarPoint(153,232),OneDollarPoint(160,233),OneDollarPoint(170,234),OneDollarPoint(177,235),OneDollarPoint(179,236),OneDollarPoint(186,237),OneDollarPoint(193,238),OneDollarPoint(198,239),OneDollarPoint(200,237),OneDollarPoint(202,239),OneDollarPoint(204,238),OneDollarPoint(206,234),OneDollarPoint(205,230),OneDollarPoint(202,222),OneDollarPoint(197,216),OneDollarPoint(192,207),OneDollarPoint(186,198),OneDollarPoint(179,189),OneDollarPoint(174,183),OneDollarPoint(170,178),OneDollarPoint(164,171),OneDollarPoint(161,168),OneDollarPoint(154,160),OneDollarPoint(148,155),OneDollarPoint(143,150),OneDollarPoint(138,148),OneDollarPoint(136,148) };
        addTemplate("triangle", trianglePts, 66);
	OneDollarPoint xPts[51] = { OneDollarPoint(87,142),OneDollarPoint(89,145),OneDollarPoint(91,148),OneDollarPoint(93,151),OneDollarPoint(96,155),OneDollarPoint(98,157),OneDollarPoint(100,160),OneDollarPoint(102,162),OneDollarPoint(106,167),OneDollarPoint(108,169),OneDollarPoint(110,171),OneDollarPoint(115,177),OneDollarPoint(119,183),OneDollarPoint(123,189),OneDollarPoint(127,193),OneDollarPoint(129,196),OneDollarPoint(133,200),OneDollarPoint(137,206),OneDollarPoint(140,209),OneDollarPoint(143,212),OneDollarPoint(146,215),OneDollarPoint(151,220),OneDollarPoint(153,222),OneDollarPoint(155,223),OneDollarPoint(157,225),OneDollarPoint(158,223),OneDollarPoint(157,218),OneDollarPoint(155,211),OneDollarPoint(154,208),OneDollarPoint(152,200),OneDollarPoint(150,189),OneDollarPoint(148,179),OneDollarPoint(147,170),OneDollarPoint(147,158),OneDollarPoint(147,148),OneDollarPoint(147,141),OneDollarPoint(147,136),OneDollarPoint(144,135),OneDollarPoint(142,137),OneDollarPoint(140,139),OneDollarPoint(135,145),OneDollarPoint(131,152),OneDollarPoint(124,163),OneDollarPoint(116,177),OneDollarPoint(108,191),OneDollarPoint(100,206),OneDollarPoint(94,217),OneDollarPoint(91,222),OneDollarPoint(89,225),OneDollarPoint(87,226),OneDollarPoint(87,224) };
        addTemplate("x", xPts, 51);
	OneDollarPoint rectanglePts[82] = { OneDollarPoint(78,149),OneDollarPoint(78,153),OneDollarPoint(78,157),OneDollarPoint(78,160),OneDollarPoint(79,162),OneDollarPoint(79,164),OneDollarPoint(79,167),OneDollarPoint(79,169),OneDollarPoint(79,173),OneDollarPoint(79,178),OneDollarPoint(79,183),OneDollarPoint(80,189),OneDollarPoint(80,193),OneDollarPoint(80,198),OneDollarPoint(80,202),OneDollarPoint(81,208),OneDollarPoint(81,210),OneDollarPoint(81,216),OneDollarPoint(82,222),OneDollarPoint(82,224),OneDollarPoint(82,227),OneDollarPoint(83,229),OneDollarPoint(83,231),OneDollarPoint(85,230),OneDollarPoint(88,232),OneDollarPoint(90,233),OneDollarPoint(92,232),OneDollarPoint(94,233),OneDollarPoint(99,232),OneDollarPoint(102,233),OneDollarPoint(106,233),OneDollarPoint(109,234),OneDollarPoint(117,235),OneDollarPoint(123,236),OneDollarPoint(126,236),OneDollarPoint(135,237),OneDollarPoint(142,238),OneDollarPoint(145,238),OneDollarPoint(152,238),OneDollarPoint(154,239),OneDollarPoint(165,238),OneDollarPoint(174,237),OneDollarPoint(179,236),OneDollarPoint(186,235),OneDollarPoint(191,235),OneDollarPoint(195,233),OneDollarPoint(197,233),OneDollarPoint(200,233),OneDollarPoint(201,235),OneDollarPoint(201,233),OneDollarPoint(199,231),OneDollarPoint(198,226),OneDollarPoint(198,220),OneDollarPoint(196,207),OneDollarPoint(195,195),OneDollarPoint(195,181),OneDollarPoint(195,173),OneDollarPoint(195,163),OneDollarPoint(194,155),OneDollarPoint(192,145),OneDollarPoint(192,143),OneDollarPoint(192,138),OneDollarPoint(191,135),OneDollarPoint(191,133),OneDollarPoint(191,130),OneDollarPoint(190,128),OneDollarPoint(188,129),OneDollarPoint(186,129),OneDollarPoint(181,132),OneDollarPoint(173,131),OneDollarPoint(162,131),OneDollarPoint(151,132),OneDollarPoint(149,132),OneDollarPoint(138,132),OneDollarPoint(136,132),OneDollarPoint(122,131),OneDollarPoint(120,131),OneDollarPoint(109,130),OneDollarPoint(107,130),OneDollarPoint(90,132),OneDollarPoint(81,133),OneDollarPoint(76,133) };
        addTemplate("rectangle", rectanglePts, 82);
	OneDollarPoint circlePts[44] = { OneDollarPoint(127,141),OneDollarPoint(124,140),OneDollarPoint(120,139),OneDollarPoint(118,139),OneDollarPoint(116,139),OneDollarPoint(111,140),OneDollarPoint(109,141),OneDollarPoint(104,144),OneDollarPoint(100,147),OneDollarPoint(96,152),OneDollarPoint(93,157),OneDollarPoint(90,163),OneDollarPoint(87,169),OneDollarPoint(85,175),OneDollarPoint(83,181),OneDollarPoint(82,190),OneDollarPoint(82,195),OneDollarPoint(83,200),OneDollarPoint(84,205),OneDollarPoint(88,213),OneDollarPoint(91,216),OneDollarPoint(96,219),OneDollarPoint(103,222),OneDollarPoint(108,224),OneDollarPoint(111,224),OneDollarPoint(120,224),OneDollarPoint(133,223),OneDollarPoint(142,222),OneDollarPoint(152,218),OneDollarPoint(160,214),OneDollarPoint(167,210),OneDollarPoint(173,204),OneDollarPoint(178,198),OneDollarPoint(179,196),OneDollarPoint(182,188),OneDollarPoint(182,177),OneDollarPoint(178,167),OneDollarPoint(170,150),OneDollarPoint(163,138),OneDollarPoint(152,130),OneDollarPoint(143,129),OneDollarPoint(140,131),OneDollarPoint(129,136),OneDollarPoint(126,139) };
        addTemplate("circle", circlePts, 44);
	OneDollarPoint checkPts[49] = { OneDollarPoint(91,185),OneDollarPoint(93,185),OneDollarPoint(95,185),OneDollarPoint(97,185),OneDollarPoint(100,188),OneDollarPoint(102,189),OneDollarPoint(104,190),OneDollarPoint(106,193),OneDollarPoint(108,195),OneDollarPoint(110,198),OneDollarPoint(112,201),OneDollarPoint(114,204),OneDollarPoint(115,207),OneDollarPoint(117,210),OneDollarPoint(118,212),OneDollarPoint(120,214),OneDollarPoint(121,217),OneDollarPoint(122,219),OneDollarPoint(123,222),OneDollarPoint(124,224),OneDollarPoint(126,226),OneDollarPoint(127,229),OneDollarPoint(129,231),OneDollarPoint(130,233),OneDollarPoint(129,231),OneDollarPoint(129,228),OneDollarPoint(129,226),OneDollarPoint(129,224),OneDollarPoint(129,221),OneDollarPoint(129,218),OneDollarPoint(129,212),OneDollarPoint(129,208),OneDollarPoint(130,198),OneDollarPoint(132,189),OneDollarPoint(134,182),OneDollarPoint(137,173),OneDollarPoint(143,164),OneDollarPoint(147,157),OneDollarPoint(151,151),OneDollarPoint(155,144),OneDollarPoint(161,137),OneDollarPoint(165,131),OneDollarPoint(171,122),OneDollarPoint(174,118),OneDollarPoint(176,114),OneDollarPoint(177,112),OneDollarPoint(177,114),OneDollarPoint(175,116),OneDollarPoint(173,118) };
        addTemplate("check", checkPts, 49);
	OneDollarPoint caretPts[54] = { OneDollarPoint(79,245),OneDollarPoint(79,242),OneDollarPoint(79,239),OneDollarPoint(80,237),OneDollarPoint(80,234),OneDollarPoint(81,232),OneDollarPoint(82,230),OneDollarPoint(84,224),OneDollarPoint(86,220),OneDollarPoint(86,218),OneDollarPoint(87,216),OneDollarPoint(88,213),OneDollarPoint(90,207),OneDollarPoint(91,202),OneDollarPoint(92,200),OneDollarPoint(93,194),OneDollarPoint(94,192),OneDollarPoint(96,189),OneDollarPoint(97,186),OneDollarPoint(100,179),OneDollarPoint(102,173),OneDollarPoint(105,165),OneDollarPoint(107,160),OneDollarPoint(109,158),OneDollarPoint(112,151),OneDollarPoint(115,144),OneDollarPoint(117,139),OneDollarPoint(119,136),OneDollarPoint(119,134),OneDollarPoint(120,132),OneDollarPoint(121,129),OneDollarPoint(122,127),OneDollarPoint(124,125),OneDollarPoint(126,124),OneDollarPoint(129,125),OneDollarPoint(131,127),OneDollarPoint(132,130),OneDollarPoint(136,139),OneDollarPoint(141,154),OneDollarPoint(145,166),OneDollarPoint(151,182),OneDollarPoint(156,193),OneDollarPoint(157,196),OneDollarPoint(161,209),OneDollarPoint(162,211),OneDollarPoint(167,223),OneDollarPoint(169,229),OneDollarPoint(170,231),OneDollarPoint(173,237),OneDollarPoint(176,242),OneDollarPoint(177,244),OneDollarPoint(179,250),OneDollarPoint(181,255),OneDollarPoint(182,257) };
        addTemplate("caret", caretPts, 54);
	OneDollarPoint questionPts[63] = { OneDollarPoint(104,145),OneDollarPoint(103,142),OneDollarPoint(103,140),OneDollarPoint(103,138),OneDollarPoint(103,135),OneDollarPoint(104,133),OneDollarPoint(105,131),OneDollarPoint(106,128),OneDollarPoint(107,125),OneDollarPoint(108,123),OneDollarPoint(111,121),OneDollarPoint(113,118),OneDollarPoint(115,116),OneDollarPoint(117,116),OneDollarPoint(119,116),OneDollarPoint(121,115),OneDollarPoint(124,116),OneDollarPoint(126,115),OneDollarPoint(128,114),OneDollarPoint(130,115),OneDollarPoint(133,116),OneDollarPoint(135,117),OneDollarPoint(140,120),OneDollarPoint(142,121),OneDollarPoint(144,123),OneDollarPoint(146,125),OneDollarPoint(149,127),OneDollarPoint(150,129),OneDollarPoint(152,130),OneDollarPoint(154,132),OneDollarPoint(156,134),OneDollarPoint(158,137),OneDollarPoint(159,139),OneDollarPoint(160,141),OneDollarPoint(160,143),OneDollarPoint(160,146),OneDollarPoint(160,149),OneDollarPoint(159,153),OneDollarPoint(158,155),OneDollarPoint(157,157),OneDollarPoint(155,159),OneDollarPoint(153,161),OneDollarPoint(151,163),OneDollarPoint(146,167),OneDollarPoint(142,170),OneDollarPoint(138,172),OneDollarPoint(134,173),OneDollarPoint(132,175),OneDollarPoint(127,175),OneDollarPoint(124,175),OneDollarPoint(122,176),OneDollarPoint(120,178),OneDollarPoint(119,180),OneDollarPoint(119,183),OneDollarPoint(119,185),OneDollarPoint(120,190),OneDollarPoint(121,194),OneDollarPoint(122,200),OneDollarPoint(123,205),OneDollarPoint(123,211),OneDollarPoint(124,215),OneDollarPoint(124,223),OneDollarPoint(124,225) };
        addTemplate("question", questionPts, 63);
	OneDollarPoint arrowPts[66] = { OneDollarPoint(68,222),OneDollarPoint(70,220),OneDollarPoint(73,218),OneDollarPoint(75,217),OneDollarPoint(77,215),OneDollarPoint(80,213),OneDollarPoint(82,212),OneDollarPoint(84,210),OneDollarPoint(87,209),OneDollarPoint(89,208),OneDollarPoint(92,206),OneDollarPoint(95,204),OneDollarPoint(101,201),OneDollarPoint(106,198),OneDollarPoint(112,194),OneDollarPoint(118,191),OneDollarPoint(124,187),OneDollarPoint(127,186),OneDollarPoint(132,183),OneDollarPoint(138,181),OneDollarPoint(141,180),OneDollarPoint(146,178),OneDollarPoint(154,173),OneDollarPoint(159,171),OneDollarPoint(161,170),OneDollarPoint(166,167),OneDollarPoint(168,167),OneDollarPoint(171,166),OneDollarPoint(174,164),OneDollarPoint(177,162),OneDollarPoint(180,160),OneDollarPoint(182,158),OneDollarPoint(183,156),OneDollarPoint(181,154),OneDollarPoint(178,153),OneDollarPoint(171,153),OneDollarPoint(164,153),OneDollarPoint(160,153),OneDollarPoint(150,154),OneDollarPoint(147,155),OneDollarPoint(141,157),OneDollarPoint(137,158),OneDollarPoint(135,158),OneDollarPoint(137,158),OneDollarPoint(140,157),OneDollarPoint(143,156),OneDollarPoint(151,154),OneDollarPoint(160,152),OneDollarPoint(170,149),OneDollarPoint(179,147),OneDollarPoint(185,145),OneDollarPoint(192,144),OneDollarPoint(196,144),OneDollarPoint(198,144),OneDollarPoint(200,144),OneDollarPoint(201,147),OneDollarPoint(199,149),OneDollarPoint(194,157),OneDollarPoint(191,160),OneDollarPoint(186,167),OneDollarPoint(180,176),OneDollarPoint(177,179),OneDollarPoint(171,187),OneDollarPoint(169,189),OneDollarPoint(165,194),OneDollarPoint(164,196) };
        addTemplate("arrow", arrowPts, 66);
	OneDollarPoint lbracketPts[76] = { OneDollarPoint(140,124),OneDollarPoint(138,123),OneDollarPoint(135,122),OneDollarPoint(133,123),OneDollarPoint(130,123),OneDollarPoint(128,124),OneDollarPoint(125,125),OneDollarPoint(122,124),OneDollarPoint(120,124),OneDollarPoint(118,124),OneDollarPoint(116,125),OneDollarPoint(113,125),OneDollarPoint(111,125),OneDollarPoint(108,124),OneDollarPoint(106,125),OneDollarPoint(104,125),OneDollarPoint(102,124),OneDollarPoint(100,123),OneDollarPoint(98,123),OneDollarPoint(95,124),OneDollarPoint(93,123),OneDollarPoint(90,124),OneDollarPoint(88,124),OneDollarPoint(85,125),OneDollarPoint(83,126),OneDollarPoint(81,127),OneDollarPoint(81,129),OneDollarPoint(82,131),OneDollarPoint(82,134),OneDollarPoint(83,138),OneDollarPoint(84,141),OneDollarPoint(84,144),OneDollarPoint(85,148),OneDollarPoint(85,151),OneDollarPoint(86,156),OneDollarPoint(86,160),OneDollarPoint(86,164),OneDollarPoint(86,168),OneDollarPoint(87,171),OneDollarPoint(87,175),OneDollarPoint(87,179),OneDollarPoint(87,182),OneDollarPoint(87,186),OneDollarPoint(88,188),OneDollarPoint(88,195),OneDollarPoint(88,198),OneDollarPoint(88,201),OneDollarPoint(88,207),OneDollarPoint(89,211),OneDollarPoint(89,213),OneDollarPoint(89,217),OneDollarPoint(89,222),OneDollarPoint(88,225),OneDollarPoint(88,229),OneDollarPoint(88,231),OneDollarPoint(88,233),OneDollarPoint(88,235),OneDollarPoint(89,237),OneDollarPoint(89,240),OneDollarPoint(89,242),OneDollarPoint(91,241),OneDollarPoint(94,241),OneDollarPoint(96,240),OneDollarPoint(98,239),OneDollarPoint(105,240),OneDollarPoint(109,240),OneDollarPoint(113,239),OneDollarPoint(116,240),OneDollarPoint(121,239),OneDollarPoint(130,240),OneDollarPoint(136,237),OneDollarPoint(139,237),OneDollarPoint(144,238),OneDollarPoint(151,237),OneDollarPoint(157,236),OneDollarPoint(159,237) };
        addTemplate("lbracket", lbracketPts, 76);
	OneDollarPoint rbracketPts[45] = { OneDollarPoint(112,138),OneDollarPoint(112,136),OneDollarPoint(115,136),OneDollarPoint(118,137),OneDollarPoint(120,136),OneDollarPoint(123,136),OneDollarPoint(125,136),OneDollarPoint(128,136),OneDollarPoint(131,136),OneDollarPoint(134,135),OneDollarPoint(137,135),OneDollarPoint(140,134),OneDollarPoint(143,133),OneDollarPoint(145,132),OneDollarPoint(147,132),OneDollarPoint(149,132),OneDollarPoint(152,132),OneDollarPoint(153,134),OneDollarPoint(154,137),OneDollarPoint(155,141),OneDollarPoint(156,144),OneDollarPoint(157,152),OneDollarPoint(158,161),OneDollarPoint(160,170),OneDollarPoint(162,182),OneDollarPoint(164,192),OneDollarPoint(166,200),OneDollarPoint(167,209),OneDollarPoint(168,214),OneDollarPoint(168,216),OneDollarPoint(169,221),OneDollarPoint(169,223),OneDollarPoint(169,228),OneDollarPoint(169,231),OneDollarPoint(166,233),OneDollarPoint(164,234),OneDollarPoint(161,235),OneDollarPoint(155,236),OneDollarPoint(147,235),OneDollarPoint(140,233),OneDollarPoint(131,233),OneDollarPoint(124,233),OneDollarPoint(117,235),OneDollarPoint(114,238),OneDollarPoint(112,238) };
        addTemplate("rbracket", rbracketPts, 45);
	OneDollarPoint vPts[47] = { OneDollarPoint(89,164),OneDollarPoint(90,162),OneDollarPoint(92,162),OneDollarPoint(94,164),OneDollarPoint(95,166),OneDollarPoint(96,169),OneDollarPoint(97,171),OneDollarPoint(99,175),OneDollarPoint(101,178),OneDollarPoint(103,182),OneDollarPoint(106,189),OneDollarPoint(108,194),OneDollarPoint(111,199),OneDollarPoint(114,204),OneDollarPoint(117,209),OneDollarPoint(119,214),OneDollarPoint(122,218),OneDollarPoint(124,222),OneDollarPoint(126,225),OneDollarPoint(128,228),OneDollarPoint(130,229),OneDollarPoint(133,233),OneDollarPoint(134,236),OneDollarPoint(136,239),OneDollarPoint(138,240),OneDollarPoint(139,242),OneDollarPoint(140,244),OneDollarPoint(142,242),OneDollarPoint(142,240),OneDollarPoint(142,237),OneDollarPoint(143,235),OneDollarPoint(143,233),OneDollarPoint(145,229),OneDollarPoint(146,226),OneDollarPoint(148,217),OneDollarPoint(149,208),OneDollarPoint(149,205),OneDollarPoint(151,196),OneDollarPoint(151,193),OneDollarPoint(153,182),OneDollarPoint(155,172),OneDollarPoint(157,165),OneDollarPoint(159,160),OneDollarPoint(162,155),OneDollarPoint(164,150),OneDollarPoint(165,148),OneDollarPoint(166,146) };
        addTemplate("v", vPts, 47);
	OneDollarPoint deletePts[53] = { OneDollarPoint(123,129),OneDollarPoint(123,131),OneDollarPoint(124,133),OneDollarPoint(125,136),OneDollarPoint(127,140),OneDollarPoint(129,142),OneDollarPoint(133,148),OneDollarPoint(137,154),OneDollarPoint(143,158),OneDollarPoint(145,161),OneDollarPoint(148,164),OneDollarPoint(153,170),OneDollarPoint(158,176),OneDollarPoint(160,178),OneDollarPoint(164,183),OneDollarPoint(168,188),OneDollarPoint(171,191),OneDollarPoint(175,196),OneDollarPoint(178,200),OneDollarPoint(180,202),OneDollarPoint(181,205),OneDollarPoint(184,208),OneDollarPoint(186,210),OneDollarPoint(187,213),OneDollarPoint(188,215),OneDollarPoint(186,212),OneDollarPoint(183,211),OneDollarPoint(177,208),OneDollarPoint(169,206),OneDollarPoint(162,205),OneDollarPoint(154,207),OneDollarPoint(145,209),OneDollarPoint(137,210),OneDollarPoint(129,214),OneDollarPoint(122,217),OneDollarPoint(118,218),OneDollarPoint(111,221),OneDollarPoint(109,222),OneDollarPoint(110,219),OneDollarPoint(112,217),OneDollarPoint(118,209),OneDollarPoint(120,207),OneDollarPoint(128,196),OneDollarPoint(135,187),OneDollarPoint(138,183),OneDollarPoint(148,167),OneDollarPoint(157,153),OneDollarPoint(163,145),OneDollarPoint(165,142),OneDollarPoint(172,133),OneDollarPoint(177,127),OneDollarPoint(179,127),OneDollarPoint(180,125) };
        addTemplate("delete", deletePts, 53);
	OneDollarPoint lbracePts[58] = { OneDollarPoint(150,116),OneDollarPoint(147,117),OneDollarPoint(145,116),OneDollarPoint(142,116),OneDollarPoint(139,117),OneDollarPoint(136,117),OneDollarPoint(133,118),OneDollarPoint(129,121),OneDollarPoint(126,122),OneDollarPoint(123,123),OneDollarPoint(120,125),OneDollarPoint(118,127),OneDollarPoint(115,128),OneDollarPoint(113,129),OneDollarPoint(112,131),OneDollarPoint(113,134),OneDollarPoint(115,134),OneDollarPoint(117,135),OneDollarPoint(120,135),OneDollarPoint(123,137),OneDollarPoint(126,138),OneDollarPoint(129,140),OneDollarPoint(135,143),OneDollarPoint(137,144),OneDollarPoint(139,147),OneDollarPoint(141,149),OneDollarPoint(140,152),OneDollarPoint(139,155),OneDollarPoint(134,159),OneDollarPoint(131,161),OneDollarPoint(124,166),OneDollarPoint(121,166),OneDollarPoint(117,166),OneDollarPoint(114,167),OneDollarPoint(112,166),OneDollarPoint(114,164),OneDollarPoint(116,163),OneDollarPoint(118,163),OneDollarPoint(120,162),OneDollarPoint(122,163),OneDollarPoint(125,164),OneDollarPoint(127,165),OneDollarPoint(129,166),OneDollarPoint(130,168),OneDollarPoint(129,171),OneDollarPoint(127,175),OneDollarPoint(125,179),OneDollarPoint(123,184),OneDollarPoint(121,190),OneDollarPoint(120,194),OneDollarPoint(119,199),OneDollarPoint(120,202),OneDollarPoint(123,207),OneDollarPoint(127,211),OneDollarPoint(133,215),OneDollarPoint(142,219),OneDollarPoint(148,220),OneDollarPoint(151,221) };
        addTemplate("lbrace", lbracePts, 58);
	OneDollarPoint rbracePts[70] = { OneDollarPoint(117,132),OneDollarPoint(115,132),OneDollarPoint(115,129),OneDollarPoint(117,129),OneDollarPoint(119,128),OneDollarPoint(122,127),OneDollarPoint(125,127),OneDollarPoint(127,127),OneDollarPoint(130,127),OneDollarPoint(133,129),OneDollarPoint(136,129),OneDollarPoint(138,130),OneDollarPoint(140,131),OneDollarPoint(143,134),OneDollarPoint(144,136),OneDollarPoint(145,139),OneDollarPoint(145,142),OneDollarPoint(145,145),OneDollarPoint(145,147),OneDollarPoint(145,149),OneDollarPoint(144,152),OneDollarPoint(142,157),OneDollarPoint(141,160),OneDollarPoint(139,163),OneDollarPoint(137,166),OneDollarPoint(135,167),OneDollarPoint(133,169),OneDollarPoint(131,172),OneDollarPoint(128,173),OneDollarPoint(126,176),OneDollarPoint(125,178),OneDollarPoint(125,180),OneDollarPoint(125,182),OneDollarPoint(126,184),OneDollarPoint(128,187),OneDollarPoint(130,187),OneDollarPoint(132,188),OneDollarPoint(135,189),OneDollarPoint(140,189),OneDollarPoint(145,189),OneDollarPoint(150,187),OneDollarPoint(155,186),OneDollarPoint(157,185),OneDollarPoint(159,184),OneDollarPoint(156,185),OneDollarPoint(154,185),OneDollarPoint(149,185),OneDollarPoint(145,187),OneDollarPoint(141,188),OneDollarPoint(136,191),OneDollarPoint(134,191),OneDollarPoint(131,192),OneDollarPoint(129,193),OneDollarPoint(129,195),OneDollarPoint(129,197),OneDollarPoint(131,200),OneDollarPoint(133,202),OneDollarPoint(136,206),OneDollarPoint(139,211),OneDollarPoint(142,215),OneDollarPoint(145,220),OneDollarPoint(147,225),OneDollarPoint(148,231),OneDollarPoint(147,239),OneDollarPoint(144,244),OneDollarPoint(139,248),OneDollarPoint(134,250),OneDollarPoint(126,253),OneDollarPoint(119,253),OneDollarPoint(115,253) };
        addTemplate("rbrace", rbracePts, 70);
	OneDollarPoint starPts[109] = { OneDollarPoint(75,250),OneDollarPoint(75,247),OneDollarPoint(77,244),OneDollarPoint(78,242),OneDollarPoint(79,239),OneDollarPoint(80,237),OneDollarPoint(82,234),OneDollarPoint(82,232),OneDollarPoint(84,229),OneDollarPoint(85,225),OneDollarPoint(87,222),OneDollarPoint(88,219),OneDollarPoint(89,216),OneDollarPoint(91,212),OneDollarPoint(92,208),OneDollarPoint(94,204),OneDollarPoint(95,201),OneDollarPoint(96,196),OneDollarPoint(97,194),OneDollarPoint(98,191),OneDollarPoint(100,185),OneDollarPoint(102,178),OneDollarPoint(104,173),OneDollarPoint(104,171),OneDollarPoint(105,164),OneDollarPoint(106,158),OneDollarPoint(107,156),OneDollarPoint(107,152),OneDollarPoint(108,145),OneDollarPoint(109,141),OneDollarPoint(110,139),OneDollarPoint(112,133),OneDollarPoint(113,131),OneDollarPoint(116,127),OneDollarPoint(117,125),OneDollarPoint(119,122),OneDollarPoint(121,121),OneDollarPoint(123,120),OneDollarPoint(125,122),OneDollarPoint(125,125),OneDollarPoint(127,130),OneDollarPoint(128,133),OneDollarPoint(131,143),OneDollarPoint(136,153),OneDollarPoint(140,163),OneDollarPoint(144,172),OneDollarPoint(145,175),OneDollarPoint(151,189),OneDollarPoint(156,201),OneDollarPoint(161,213),OneDollarPoint(166,225),OneDollarPoint(169,233),OneDollarPoint(171,236),OneDollarPoint(174,243),OneDollarPoint(177,247),OneDollarPoint(178,249),OneDollarPoint(179,251),OneDollarPoint(180,253),OneDollarPoint(180,255),OneDollarPoint(179,257),OneDollarPoint(177,257),OneDollarPoint(174,255),OneDollarPoint(169,250),OneDollarPoint(164,247),OneDollarPoint(160,245),OneDollarPoint(149,238),OneDollarPoint(138,230),OneDollarPoint(127,221),OneDollarPoint(124,220),OneDollarPoint(112,212),OneDollarPoint(110,210),OneDollarPoint(96,201),OneDollarPoint(84,195),OneDollarPoint(74,190),OneDollarPoint(64,182),OneDollarPoint(55,175),OneDollarPoint(51,172),OneDollarPoint(49,170),OneDollarPoint(51,169),OneDollarPoint(56,169),OneDollarPoint(66,169),OneDollarPoint(78,168),OneDollarPoint(92,166),OneDollarPoint(107,164),OneDollarPoint(123,161),OneDollarPoint(140,162),OneDollarPoint(156,162),OneDollarPoint(171,160),OneDollarPoint(173,160),OneDollarPoint(186,160),OneDollarPoint(195,160),OneDollarPoint(198,161),OneDollarPoint(203,163),OneDollarPoint(208,163),OneDollarPoint(206,164),OneDollarPoint(200,167),OneDollarPoint(187,172),OneDollarPoint(174,179),OneDollarPoint(172,181),OneDollarPoint(153,192),OneDollarPoint(137,201),OneDollarPoint(123,211),OneDollarPoint(112,220),OneDollarPoint(99,229),OneDollarPoint(90,237),OneDollarPoint(80,244),OneDollarPoint(73,250),OneDollarPoint(69,254),OneDollarPoint(69,252) };
        addTemplate("star", starPts, 109);
	OneDollarPoint pigtailPts[65] = { OneDollarPoint(81,219),OneDollarPoint(84,218),OneDollarPoint(86,220),OneDollarPoint(88,220),OneDollarPoint(90,220),OneDollarPoint(92,219),OneDollarPoint(95,220),OneDollarPoint(97,219),OneDollarPoint(99,220),OneDollarPoint(102,218),OneDollarPoint(105,217),OneDollarPoint(107,216),OneDollarPoint(110,216),OneDollarPoint(113,214),OneDollarPoint(116,212),OneDollarPoint(118,210),OneDollarPoint(121,208),OneDollarPoint(124,205),OneDollarPoint(126,202),OneDollarPoint(129,199),OneDollarPoint(132,196),OneDollarPoint(136,191),OneDollarPoint(139,187),OneDollarPoint(142,182),OneDollarPoint(144,179),OneDollarPoint(146,174),OneDollarPoint(148,170),OneDollarPoint(149,168),OneDollarPoint(151,162),OneDollarPoint(152,160),OneDollarPoint(152,157),OneDollarPoint(152,155),OneDollarPoint(152,151),OneDollarPoint(152,149),OneDollarPoint(152,146),OneDollarPoint(149,142),OneDollarPoint(148,139),OneDollarPoint(145,137),OneDollarPoint(141,135),OneDollarPoint(139,135),OneDollarPoint(134,136),OneDollarPoint(130,140),OneDollarPoint(128,142),OneDollarPoint(126,145),OneDollarPoint(122,150),OneDollarPoint(119,158),OneDollarPoint(117,163),OneDollarPoint(115,170),OneDollarPoint(114,175),OneDollarPoint(117,184),OneDollarPoint(120,190),OneDollarPoint(125,199),OneDollarPoint(129,203),OneDollarPoint(133,208),OneDollarPoint(138,213),OneDollarPoint(145,215),OneDollarPoint(155,218),OneDollarPoint(164,219),OneDollarPoint(166,219),OneDollarPoint(177,219),OneDollarPoint(182,218),OneDollarPoint(192,216),OneDollarPoint(196,213),OneDollarPoint(199,212),OneDollarPoint(201,211) };
        addTemplate("pigtail", pigtailPts, 65);
	*/
}

Result Recognizer::Recognize(vector<OneDollarPoint> points)
{
    Result error("NONE", 0, -1);
    if (m_templates.size() == 0) return error;

    points = Resample(points, NumPoints);
    points = RotateToZero(points);
    points = ScaleToSquare(points, GESTURE_SQUARE_SIZE);
    points = TranslateToOrigin(points);

    double b = FLT_MAX;
    int t = 0;
    for (uint i = 0; i < m_templates.size(); i++)
    {
        double d = DistanceAtBestAngle(points, m_templates[i], -AngleRange, +AngleRange, AnglePrecision);
        if (d < b)
        {
            b = d;
            t = i;
        }
    }
    double score = 1.0 - (b / HalfDiagonal);
    Result r(m_templates[t].m_name, score, t);
    return r;
}

Result Recognizer::BackRecognize(vector<OneDollarPoint> points)
{
    Result r("NONE", 0, -1);
    if (m_templates.size() == 0) return r;

    // Start by looking at the last GESTURE_MIN_TRAJECTORY_LENGTH points
    // then increase the amount of points be try to recognize, going backwards, until we get a good match
    // give up when we are looking at GESTURE_MAX_TRAJECTORY_LENGTH points

    reverse(points.begin(), points.end());
    double maxScore = 0;
    int maxLength = min(int(points.size()), GESTURE_MAX_TRAJECTORY_LENGTH);
    for (int npts=GESTURE_MIN_TRAJECTORY_LENGTH; npts<maxLength; npts += GESTURE_BACKREC_STEPSIZE)
    {
        vector<OneDollarPoint> sublist;
        for (int pi=0; pi<npts; pi++)
        {
            sublist.push_back(points[pi]);
        }
        reverse(sublist.begin(), sublist.end());
        sublist = Resample(sublist, NumPoints);
        sublist = RotateToZero(sublist);
        sublist = ScaleToSquare(sublist, GESTURE_SQUARE_SIZE);
        sublist = TranslateToOrigin(sublist);

        double b = FLT_MAX;
        int t = 0;
        for (int i = m_templates.size()-1; i >=0; i--)
        {
            double d = DistanceAtBestAngle(sublist, m_templates[i], -AngleRange, +AngleRange, AnglePrecision);
            if (d < b)
            {
                b = d;
                t = i;
            }
        }
        double score = 1.0 - (b / HalfDiagonal);
        if (score > maxScore)
        {	// we've found the best result so far
            maxScore = score;
            r.m_name = m_templates[t].m_name;
            r.m_score = score;
            r.m_index = t;
        }
    }
    return r;
}

int Recognizer::addTemplate(string name, OneDollarPoint* points, int npoints)
{
    vector<OneDollarPoint> newpoints;
    for (int i=0; i<npoints; i++)
    {
        OneDollarPoint np(points[i].m_x, points[i].m_y);
        newpoints.push_back(np);
    }
    return addTemplate(name, newpoints);
}

int Recognizer::addTemplate(string name, Template t)
{
    vector<OneDollarPoint> newpoints;
    for (int i=0; i<t.GetLength(); i++)
    {
        OneDollarPoint np(t.m_points[i].m_x, t.m_points[i].m_y);
        newpoints.push_back(np);
    }
    return addTemplate(name, newpoints);
}

int Recognizer::addTemplate(string name, vector<OneDollarPoint> points)
{
    Template t(name, points);
    m_templates.push_back(t);

    int num = 0;
    for (uint i = 0; i < m_templates.size(); i++)
    {
        if (m_templates[i].m_name.compare(name) == 0) num++;
    }
    return num;
}

int Recognizer::DeleteUserTemplates()
{
    m_templates.erase(m_templates.begin() + NumTemps, m_templates.end());
    return NumTemps;
}

//
// Helper functions from this point down
//

vector<OneDollarPoint> Resample(vector<OneDollarPoint> points, int n)
{
    if (points.size() < 1)
        return points;
    double I = PathLength(points) / (n - 1); // interval length
    double D = 0.0;
    vector<OneDollarPoint> srcPts = points;
    vector<OneDollarPoint> dstPts;
    dstPts.push_back(srcPts[0]);
    for (uint i = 1; i < srcPts.size(); i++)
    {
        OneDollarPoint pt1 = srcPts[i - 1];
        OneDollarPoint pt2 = srcPts[i];
        double d = Distance(pt1, pt2);
        if ((D + d) >= I)
        {
            double qx = pt1.m_x + ((I - D) / d) * (pt2.m_x - pt1.m_x);
            double qy = pt1.m_y + ((I - D) / d) * (pt2.m_y - pt1.m_y);
            OneDollarPoint q(qx, qy);
            dstPts.push_back(q); // append new point 'q'
            srcPts.insert(srcPts.begin()+i, q); // insert 'q' at position i in points s.t. 'q' will be the next i
            D = 0.0;
        }
        else
        {
            D += d;
        }
    }
    // somtimes we fall a rounding-error short of adding the last point, so add it if so
    if (int(dstPts.size()) == (n - 1))
    {
        dstPts.push_back(srcPts[srcPts.size() - 1]);
    }

    return dstPts;
}


vector<OneDollarPoint> RotateToZero(vector<OneDollarPoint> points)
{
    if (points.size() < 1)
        return points;
    OneDollarPoint c = Centroid(points);
    double theta = atan2((double)(c.m_y - points[0].m_y), (double)(c.m_x - points[0].m_x));
    return RotateBy(points, -theta);
}

vector<OneDollarPoint> RotateBy(vector<OneDollarPoint> points, double theta)
{
    if (points.size() < 1)
        return points;
    OneDollarPoint c = Centroid(points);
    vector<OneDollarPoint> newpoints;
    for (uint i = 0; i < points.size(); i++)
    {
        double qx = (points[i].m_x - c.m_x) * cos(theta) - (points[i].m_y - c.m_y) * sin(theta) + c.m_x;
        double qy = (points[i].m_x - c.m_x) * sin(theta) + (points[i].m_y - c.m_y) * cos(theta) + c.m_y;
        OneDollarPoint np(qx, qy);
        newpoints.push_back(np);
    }
    return newpoints;
}	

vector<OneDollarPoint> ScaleToSquare(vector<OneDollarPoint> points, double size)
{
    if (points.size() < 1)
        return points;
    OneDollarRectangle B = BoundingBox(points);

    double shortside, longside, sideratio, scalex, scaley;
    if (B.m_width < B.m_height)
    {
        shortside = B.m_width;
        longside = B.m_height;
    }
    else
    {
        shortside = B.m_height;
        longside = B.m_width;
    }
    sideratio = shortside / longside;
    if (sideratio < 0.3)
    {	// we consider this a 1D gesture and scale it uniformly
        scalex = size / longside;
        scaley = size / longside;
    }
    else
    {	// this is a 2D gesture so we will scale it non-uniformly
        scalex = size / B.m_width;
        scaley = size / B.m_height;
    }

    vector<OneDollarPoint> newpoints;

    for (uint i = 0; i < points.size(); i++)
    {
        double qx = points[i].m_x * scalex;
        double qy = points[i].m_y * scaley;
        OneDollarPoint np(qx, qy);
        newpoints.push_back(np);
    }
    return newpoints;
}			

vector<OneDollarPoint> TranslateToOrigin(vector<OneDollarPoint> points)
{
    if (points.size() < 1)
        return points;
    OneDollarPoint c = Centroid(points);
    vector<OneDollarPoint> newpoints;
    for (uint i = 0; i < points.size(); i++)
    {
        double qx = points[i].m_x - c.m_x;
        double qy = points[i].m_y - c.m_y;
        OneDollarPoint np(qx, qy);
        newpoints.push_back(np);
    }
    return newpoints;
}		

double DistanceAtBestAngle(vector<OneDollarPoint> points, Template T, double a, double b, double threshold)
{
    double x1 = PHI * a + (1.0 - PHI) * b;
    double f1 = DistanceAtAngle(points, T, x1);
    double x2 = (1.0 - PHI) * a + PHI * b;
    double f2 = DistanceAtAngle(points, T, x2);
    while (fabs(b - a) > threshold)
    {
        if (f1 < f2)
        {
            b = x2;
            x2 = x1;
            f2 = f1;
            x1 = PHI * a + (1.0 - PHI) * b;
            f1 = DistanceAtAngle(points, T, x1);
        }
        else
        {
            a = x1;
            x1 = x2;
            f1 = f2;
            x2 = (1.0 - PHI) * a + PHI * b;
            f2 = DistanceAtAngle(points, T, x2);
        }
    }
    return MIN(f1, f2);
}

double DistanceAtAngle(vector<OneDollarPoint> points, Template T, double theta)
{
    vector<OneDollarPoint> newpoints = RotateBy(points, theta);
    return PathDistance(newpoints, T.m_points);
}	

OneDollarPoint Centroid(vector<OneDollarPoint> points)
{
    double x = 0.0, y = 0.0;
    for (uint i = 0; i < points.size(); i++)
    {
        x += points[i].m_x;
        y += points[i].m_y;
    }
    x /= points.size();
    y /= points.size();
    OneDollarPoint np(x, y);
    return np;
}	

OneDollarRectangle BoundingBox(vector<OneDollarPoint> points)
{
    double minX = FLT_MAX, maxX = -FLT_MAX, minY = FLT_MAX, maxY = -FLT_MAX;
    for (uint i = 0; i < points.size(); i++)
    {
        if (points[i].m_x < minX)
            minX = points[i].m_x;
        if (points[i].m_x > maxX)
            maxX = points[i].m_x;
        if (points[i].m_y < minY)
            minY = points[i].m_y;
        if (points[i].m_y > maxY)
            maxY = points[i].m_y;
    }
    OneDollarRectangle nr(minX, minY, maxX - minX, maxY - minY);
    return nr;
}	

double PathDistance(vector<OneDollarPoint> pts1, vector<OneDollarPoint> pts2)
{
    double d = 0.0;
    for (uint i = 0; i < pts1.size(); i++)
    { // assumes pts1.length == pts2.length
        d += Distance(pts1[i], pts2[i]);
    }
    return (d / pts1.size());
}

double PathLength(vector<OneDollarPoint> points)
{
    double d = 0.0;
    for (uint i = 1; i < points.size(); i++)
    {
        d += Distance(points[i - 1], points[i]);
    }
    return d;
}	

double Distance(OneDollarPoint p1, OneDollarPoint p2)
{
    return hypot(p2.m_x-p1.m_x, p2.m_y-p1.m_y);
}
