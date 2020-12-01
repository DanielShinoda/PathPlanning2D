#include "xmllogger.h"
#include <iostream>

using tinyxml2::XMLElement;
using tinyxml2::XMLNode;

bool XmlLogger::getLog(const char* FileName, const std::string* LogParams)
{
    if (loglevel == CN_LP_LEVEL_NOPE_WORD) return true;

    if (doc.LoadFile(FileName) != tinyxml2::XMLError::XML_SUCCESS) {
        std::cout << "Error opening input XML file" << std::endl;
        return false;
    }

    if (LogParams[CN_LP_PATH] == "" && LogParams[CN_LP_NAME] == "") {
        std::string str;
        str.append(FileName);
        size_t found = str.find_last_of(".");
        if (found != std::string::npos)
            str.insert(found, "_log");
        else
            str.append("_log");
        LogFileName.append(str);
    }
    else if (LogParams[CN_LP_PATH] == "") {
        LogFileName.append(FileName);
        std::string::iterator it = LogFileName.end();
        while (*it != '\\')
            it--;
        ++it;
        LogFileName.erase(it, LogFileName.end());
        LogFileName.append(LogParams[CN_LP_NAME]);
    }
    else if (LogParams[CN_LP_NAME] == "") {
        LogFileName.append(LogParams[CN_LP_PATH]);
        if (*(--LogParams[CN_LP_PATH].end()) != '\\') LogFileName.append("\\");
        std::string lfn;
        lfn.append(FileName);
        size_t found = lfn.find_last_of("\\");
        std::string str = lfn.substr(found);
        found = str.find_last_of(".");
        if (found != std::string::npos)
            str.insert(found, "_log");
        else
            str.append("_log");
        LogFileName.append(str);
    }
    else {
        LogFileName.append(LogParams[CN_LP_PATH]);
        if (*(--LogParams[CN_LP_PATH].end()) != '\\') LogFileName.append("\\");
        LogFileName.append(LogParams[CN_LP_NAME]);
    }

    XMLElement* log, * root = doc.FirstChildElement(CNS_TAG_ROOT);

    if (!root) {
        std::cout << "No '" << CNS_TAG_ROOT << "' element found in XML file" << std::endl;
        std::cout << "Can not create log" << std::endl;
        return false;
    }

    root->InsertEndChild(doc.NewElement(CNS_TAG_LOG));

    root = (root->LastChild())->ToElement();

    if (loglevel != CN_LP_LEVEL_NOPE_WORD) {
        log = doc.NewElement(CNS_TAG_MAPFN);
        log->LinkEndChild(doc.NewText(FileName));
        root->InsertEndChild(log);

        root->InsertEndChild(doc.NewElement(CNS_TAG_SUM));

        root->InsertEndChild(doc.NewElement(CNS_TAG_PATH));

        root->InsertEndChild(doc.NewElement(CNS_TAG_LPLEVEL));

        root->InsertEndChild(doc.NewElement(CNS_TAG_HPLEVEL));
    }

    if (loglevel == CN_LP_LEVEL_FULL_WORD || loglevel == CN_LP_LEVEL_MEDIUM_WORD)
        root->InsertEndChild(doc.NewElement(CNS_TAG_LOWLEVEL));

    return true;
}

void XmlLogger::saveLog()
{
    if (loglevel == CN_LP_LEVEL_NOPE_WORD)
        return;
    doc.SaveFile(LogFileName.c_str());
}

void XmlLogger::writeToLogMap(const Map& map, const std::list<Node>& path)
{
    if (loglevel == CN_LP_LEVEL_NOPE_WORD || loglevel == CN_LP_LEVEL_TINY_WORD)
        return;

    XMLElement* mapTag = doc.FirstChildElement(CNS_TAG_ROOT);
    mapTag = mapTag->FirstChildElement(CNS_TAG_LOG)->FirstChildElement(CNS_TAG_PATH);

    int iterate = 0;
    bool inPath;
    std::string str;
    for (int i = 0; i < map.getMapHeight(); ++i) {
        XMLElement* element = doc.NewElement(CNS_TAG_ROW);
        element->SetAttribute(CNS_TAG_ATTR_NUM, iterate);

        for (int j = 0; j < map.getMapWidth(); ++j) {
            inPath = false;
            for (auto it = path.begin(); it != path.end(); it++)
                if (it->i == i && it->j == j) {
                    inPath = true;
                    break;
                }
            if (!inPath)
                str += std::to_string(map.getValue(i, j));
            else if (inPath && (i == map.getStartI() && j == map.getStartJ()))
                str += "S";
            else if (inPath && (i == map.getGoalI() && j == map.getGoalJ()))
                str += "F";
            else
                str += CNS_OTHER_PATHSELECTION;
            str += CNS_OTHER_MATRIXSEPARATOR;
            str += CNS_OTHER_PATHSELECTION;
        }

        element->InsertEndChild(doc.NewText(str.c_str()));
        mapTag->InsertEndChild(element);
        str.clear();
        iterate++;
    }
}

void XmlLogger::writeToLogOpenClose(const std::unordered_map<int, Node>& open, const std::unordered_map<int, Node>& close, bool last) {
    if (loglevel != CN_LP_LEVEL_FULL_WORD && !(loglevel == CN_LP_LEVEL_MEDIUM_WORD && last)) {
        return;
    }
    XMLElement* nextElem = nullptr;
    XMLElement* elem = doc.NewElement(CNS_TAG_STEP);
    XMLElement* lowlevel = doc.FirstChildElement(CNS_TAG_ROOT);
    lowlevel = lowlevel->FirstChildElement(CNS_TAG_LOG)->FirstChildElement(CNS_TAG_LOWLEVEL);
    int now = 0;
    for (nextElem = lowlevel->FirstChildElement(); nextElem != nullptr; nextElem = nextElem->NextSiblingElement()) {
        now++;
    }
    elem->SetAttribute(CNS_TAG_ATTR_NUM, now);
    lowlevel->InsertEndChild(elem);
    lowlevel = lowlevel->LastChildElement();
    lowlevel->InsertEndChild(doc.NewElement(CNS_TAG_OPEN));
    nextElem = lowlevel->LastChildElement();
    Node minNode;
    minNode.F = -1;
    int ind_min = 0;
    for (auto it = open.begin(); it != open.end(); ++it) {
        if (it->second.F <= minNode.F || minNode.F == -1) {
            if (it->second.F == minNode.F && it->second.g > minNode.g) {
                minNode = it->second;
                ind_min = it->first;
            }
            else if (it->second.F < minNode.F || minNode.F == -1) {
                minNode = it->second;
                ind_min = it->first;
            }
        }
    }
    if (minNode.F != -1) {
        elem = doc.NewElement(CNS_TAG_POINT);
        elem->SetAttribute(CNS_TAG_ATTR_F, minNode.F);
        elem->SetAttribute(CNS_TAG_ATTR_G, minNode.g);
        elem->SetAttribute(CNS_TAG_ATTR_Y, minNode.i);
        elem->SetAttribute(CNS_TAG_ATTR_X, minNode.j);
        if (minNode.g > 0) {
            elem->SetAttribute(CNS_TAG_ATTR_PARY, minNode.parent->i);
            elem->SetAttribute(CNS_TAG_ATTR_PARX, minNode.parent->j);
        }
        nextElem->InsertEndChild(elem);
    }
    for (auto it = open.begin(); it != open.end(); ++it) {
        if (it->first != ind_min) {
            elem = doc.NewElement(CNS_TAG_POINT);
            elem->SetAttribute(CNS_TAG_ATTR_F, it->second.F);
            elem->SetAttribute(CNS_TAG_ATTR_G, it->second.g);
            elem->SetAttribute(CNS_TAG_ATTR_Y, it->second.i);
            elem->SetAttribute(CNS_TAG_ATTR_X, it->second.j);
            if (it->second.g > 0) {
                elem->SetAttribute(CNS_TAG_ATTR_PARY, it->second.parent->i);
                elem->SetAttribute(CNS_TAG_ATTR_PARX, it->second.parent->j);
            }
            nextElem->InsertEndChild(elem);
        }
        nextElem->InsertEndChild(elem);
    }
    lowlevel->InsertEndChild(doc.NewElement(CNS_TAG_CLOSE));
    nextElem = lowlevel->LastChildElement();
    for (auto it = close.begin(); it != close.end(); ++it) {
        elem = doc.NewElement(CNS_TAG_POINT);
        elem->SetAttribute(CNS_TAG_ATTR_F, it->second.F);
        elem->SetAttribute(CNS_TAG_ATTR_G, it->second.g);
        elem->SetAttribute(CNS_TAG_ATTR_Y, it->second.i);
        elem->SetAttribute(CNS_TAG_ATTR_X, it->second.j);
        if (it->second.g > 0) {
            elem->SetAttribute(CNS_TAG_ATTR_PARY, it->second.parent->i);
            elem->SetAttribute(CNS_TAG_ATTR_PARX, it->second.parent->j);
        }
        nextElem->InsertEndChild(elem);
    }
}

void XmlLogger::writeToLogPath(const std::list<Node>& path)
{
    if (loglevel == CN_LP_LEVEL_NOPE_WORD || loglevel == CN_LP_LEVEL_TINY_WORD || path.empty())
        return;
    int iterate = 0;
    XMLElement* lplevel = doc.FirstChildElement(CNS_TAG_ROOT);
    lplevel = lplevel->FirstChildElement(CNS_TAG_LOG)->FirstChildElement(CNS_TAG_LPLEVEL);

    for (std::list<Node>::const_iterator it = path.begin(); it != path.end(); it++) {
        XMLElement* element = doc.NewElement(CNS_TAG_POINT);
        element->SetAttribute(CNS_TAG_ATTR_X, it->j);
        element->SetAttribute(CNS_TAG_ATTR_Y, it->i);
        element->SetAttribute(CNS_TAG_ATTR_NUM, iterate);
        lplevel->InsertEndChild(element);
        iterate++;
    }
}

void XmlLogger::writeToLogHPpath(const std::list<Node>& hppath)
{
    if (loglevel == CN_LP_LEVEL_NOPE_WORD || loglevel == CN_LP_LEVEL_TINY_WORD || hppath.empty())
        return;
    int partnumber = 0;
    XMLElement* hplevel = doc.FirstChildElement(CNS_TAG_ROOT);
    hplevel = hplevel->FirstChildElement(CNS_TAG_LOG)->FirstChildElement(CNS_TAG_HPLEVEL);
    std::list<Node>::const_iterator iter = hppath.begin();
    std::list<Node>::const_iterator it = hppath.begin();

    while (iter != --hppath.end()) {
        XMLElement* part = doc.NewElement(CNS_TAG_SECTION);
        part->SetAttribute(CNS_TAG_ATTR_NUM, partnumber);
        part->SetAttribute(CNS_TAG_ATTR_STX, it->j);
        part->SetAttribute(CNS_TAG_ATTR_STY, it->i);
        ++iter;
        part->SetAttribute(CNS_TAG_ATTR_FINX, iter->j);
        part->SetAttribute(CNS_TAG_ATTR_FINY, iter->i);
        part->SetAttribute(CNS_TAG_ATTR_LENGTH, iter->g - it->g);
        hplevel->LinkEndChild(part);
        ++it;
        ++partnumber;
    }
}

void XmlLogger::writeToLogSummary(unsigned int numberofsteps, unsigned int nodescreated, float length, double time, double cellSize)
{
    if (loglevel == CN_LP_LEVEL_NOPE_WORD)
        return;

    XMLElement* summary = doc.FirstChildElement(CNS_TAG_ROOT);
    summary = summary->FirstChildElement(CNS_TAG_LOG)->FirstChildElement(CNS_TAG_SUM);
    XMLElement* element = summary->ToElement();
    element->SetAttribute(CNS_TAG_ATTR_NUMOFSTEPS, numberofsteps);
    element->SetAttribute(CNS_TAG_ATTR_NODESCREATED, nodescreated);
    element->SetAttribute(CNS_TAG_ATTR_LENGTH, length);
    element->SetAttribute(CNS_TAG_ATTR_LENGTH_SCALED, length * cellSize);
    element->SetAttribute(CNS_TAG_ATTR_TIME, std::to_string(time).c_str());
}

void XmlLogger::writeToLogNotFound()
{
    if (loglevel == CN_LP_LEVEL_NOPE_WORD)
        return;

    XMLElement* node = doc.FirstChildElement(CNS_TAG_ROOT)->FirstChildElement(CNS_TAG_LOG)->FirstChildElement(CNS_TAG_PATH);
    node->InsertEndChild(doc.NewText("Path NOT found!"));
}