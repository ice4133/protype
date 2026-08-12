#ifndef YOUR_PACKAGE_XML_VECTOR_IO_HPP_
#define YOUR_PACKAGE_XML_VECTOR_IO_HPP_

#include <Eigen/Dense>
#include <tinyxml2.h>
#include <string>
#include <stdexcept>

namespace your_package {

class VectorXmlIO {
public:
    /**
     * @brief 保存 Eigen::VectorXd 到 XML 文件
     * @param filename XML 文件路径
     * @param vector 要保存的向量
     * @param vector_name 向量在 XML 中的名称
     * @return 是否保存成功
     */
    static bool saveVectorXdToXml(
        const std::string& filename, 
        const Eigen::VectorXd& vector,
        const std::string& vector_name = "VectorXd")
    {
        tinyxml2::XMLDocument doc;
        tinyxml2::XMLElement* root = doc.NewElement("EigenVector");
        doc.InsertFirstChild(root);
        
        // 添加元数据
        tinyxml2::XMLElement* meta = doc.NewElement("Metadata");
        meta->SetAttribute("name", vector_name.c_str());
        meta->SetAttribute("size", static_cast<int>(vector.size()));
        root->InsertEndChild(meta);
        
        // 添加数据元素
        tinyxml2::XMLElement* data = doc.NewElement("Data");
        for (int i = 0; i < vector.size(); ++i) {
            tinyxml2::XMLElement* element = doc.NewElement("Element");
            element->SetAttribute("index", i);
            element->SetText(vector[i]);
            data->InsertEndChild(element);
        }
        root->InsertEndChild(data);
        
        // 保存文件
        tinyxml2::XMLError error = doc.SaveFile(filename.c_str());
        return (error == tinyxml2::XML_SUCCESS);
    }

    /**
     * @brief 从 XML 文件加载 Eigen::VectorXd
     * @param filename XML 文件路径
     * @param vector_name 要加载的向量名称
     * @return 加载的向量
     * @throw std::runtime_error 如果加载失败
     */
    static Eigen::VectorXd loadVectorXdFromXml(
        const std::string& filename,
        const std::string& vector_name = "VectorXd")
    {
        tinyxml2::XMLDocument doc;
        if (doc.LoadFile(filename.c_str()) {
            throw std::runtime_error("Failed to load XML file: " + filename);
        }
        
        tinyxml2::XMLElement* root = doc.FirstChildElement("EigenVector");
        if (!root) {
            throw std::runtime_error("Invalid XML format: missing EigenVector root");
        }
        
        // 查找指定名称的向量
        tinyxml2::XMLElement* meta = root->FirstChildElement("Metadata");
        while (meta) {
            const char* name = meta->Attribute("name");
            if (name && std::string(name) == vector_name) {
                break;
            }
            meta = meta->NextSiblingElement("Metadata");
        }
        
        if (!meta) {
            throw std::runtime_error("Vector not found in XML: " + vector_name);
        }
        
        // 获取向量大小
        int size = 0;
        if (meta->QueryIntAttribute("size", &size) != tinyxml2::XML_SUCCESS || size <= 0) {
            throw std::runtime_error("Invalid vector size in XML");
        }
        
        // 创建向量
        Eigen::VectorXd vector(size);
        
        // 加载数据
        tinyxml2::XMLElement* data = root->FirstChildElement("Data");
        if (!data) {
            throw std::runtime_error("Missing data section in XML");
        }
        
        tinyxml2::XMLElement* element = data->FirstChildElement("Element");
        int count = 0;
        
        while (element && count < size) {
            int index = -1;
            if (element->QueryIntAttribute("index", &index) != tinyxml2::XML_SUCCESS || 
                index < 0 || index >= size) {
                throw std::runtime_error("Invalid element index in XML");
            }
            
            double value = 0.0;
            if (element->QueryDoubleText(&value) != tinyxml2::XML_SUCCESS) {
                throw std::runtime_error("Failed to parse element value");
            }
            
            vector[index] = value;
            element = element->NextSiblingElement("Element");
            count++;
        }
        
        if (count != size) {
            throw std::runtime_error("Incomplete data in XML");
        }
        
        return vector;
    }
};

} // namespace your_package

#endif // YOUR_PACKAGE_XML_VECTOR_IO_HPP_