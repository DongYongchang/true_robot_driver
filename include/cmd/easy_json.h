/*
 * desc: .
 * author: garin.yang
 * date: 2022/11/3
 */

#ifndef CPPBATIS_EASY_JSON_HPP
#define CPPBATIS_EASY_JSON_HPP
#include <assert.h>
#include "json.hpp"
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <iostream>

namespace Json {
    using namespace std;

    // ģ�������Ϣ��¼
    static std::string uni_err_message = "";

    /*======  Unmarshalʵ��  =======*/

    // ���һ���� �Ƿ��зǾ�̬�����ص�unmarshal����
    template <typename T>
    struct TestUnmarshalFunc {
        template <typename TT>
        static char func(decltype(&TT::unmarshal));
        template <typename TT>
        static int func(...);
        const static bool has = (sizeof(func<T>(NULL)) == sizeof(char));
        const static bool has_enum = std::is_enum<T>::value;
    };

    //������������� unmarshal ����������ö����unmarshal
    template <typename T, typename enable_if<TestUnmarshalFunc<T>::has, int>::type = 0>
    inline bool Unmarshal(T& obj, const nlohmann::json& root) {
        return obj.unmarshal(root);
    }

    //�������Ϊenum���ͣ���תΪ��Ӧ��ö������
    template <typename T, typename enable_if<TestUnmarshalFunc<T>::has_enum, int>::type = 0>
    inline bool Unmarshal(T& obj, const nlohmann::json& root) {
        int temp = root.get<int>();
        obj = static_cast<T>(temp);
        return true;
    }

    //�������������� int long bool float double string char
    template <int = 0>
    inline bool Unmarshal(char& obj, const nlohmann::json& root) {
        if (!root.is_string())
            return false;
        auto tmp = root.get<std::string>();
        assert(tmp.size() == 1);
        obj = tmp[0];
        return true;
    }

    template <int = 0>
    inline bool Unmarshal(uint16_t& obj, const nlohmann::json& root) {
        if (!root.is_number_integer())
            return false;
        obj = root.get<int16_t>();
        return true;
    }

    template <int = 0>
    inline bool Unmarshal(uint32_t& obj, const nlohmann::json& root) {
        if (!root.is_number_integer())
            return false;
        obj = root.get<uint32_t>();
        return true;
    }

    template <int = 0>
    inline bool Unmarshal(int64_t& obj, const nlohmann::json& root) {
        if (!root.is_number_integer())
            return false;
        obj = root.get<uint64_t>();
        return true;
    }

    template <int = 0>
    inline bool Unmarshal(uint64_t& obj, const nlohmann::json& root) {
        if (!root.is_number_integer())
            return false;
        obj = root.get<uint64_t>();
        return true;
    }

    template <int = 0>
    inline bool Unmarshal(int& obj, const nlohmann::json& root) {
        if (!root.is_number_integer())
            return false;
        obj = root.get<int32_t>();
        return true;
    }

    template <int = 0>
    inline bool Unmarshal(bool& obj, const nlohmann::json& root) {
        if (!root.is_boolean())
            return false;
        obj = root.get<bool>();
        return true;
    }

    template <int = 0>
    inline bool Unmarshal(float& obj, const nlohmann::json& root) {
        if (!root.is_number_float())
            return false;
        obj = root.get<float>();
        return true;
    }

    template <int = 0>
    inline bool Unmarshal(double& obj, const nlohmann::json& root) {
        if (!root.is_number_float())
            return false;
        obj = root.get<float>();
        return true;
    }

    template <int = 0>
    inline bool Unmarshal(string& obj, const nlohmann::json& root) {
        if (!root.is_string())
            return false;
        obj = root.get<string>();
        return true;
    }

    template <int = 0>
    inline bool Unmarshal(char* obj, const nlohmann::json& root) {
        if (!root.is_string())
            return false;
        strcpy(obj, root.get<std::string>().c_str());
        return true;
    }
    //������stl��������������unmarshal�����Ķ���
    template <typename T>
    bool Unmarshal(vector<T>&, const nlohmann::json&);  //��ģ���໥ѭ�����öԷ�ʱ,����������,ʹ�������͵� ģ��ɼ�!
    template <typename T>
    bool Unmarshal(map<string, T>&, const nlohmann::json&);
    template <typename T>
    bool Unmarshal(map<int, T>& obj, const nlohmann::json& root);
    template <typename T>
    bool Unmarshal(map<long, T>& obj, const nlohmann::json& root);

    // vector
    template <typename T>
    bool Unmarshal(vector<T>& obj, const nlohmann::json& root) {
#if 0
        if (!root.isArray())
            return false;
        obj.clear();
        bool ret = true;
        for (int i = 0; i < root.size(); ++i) {
            T tmp;  //����Tһ��Ҫ����Ĭ�Ϲ�������
            if (!Unmarshal(tmp, root[i]))
                ret = false;
            obj.push_back(tmp);
        }
#else
        if (!root.is_array())
            return false;
        obj.clear();
        bool ret = true;
        for (unsigned int i = 0; i < root.size(); ++i) {
            T tmp;  //����Tһ��Ҫ����Ĭ�Ϲ�������
            if (!Unmarshal(tmp, root[i]))
                ret = false;
            obj.push_back(tmp);
        }
#endif
        return ret;
    }

    // map key:string
    template <typename T>
    bool Unmarshal(map<string, T>& obj, const nlohmann::json& root) {
#if 0
        if (!root.isObject())
            return false;
        obj.clear();
        auto mems = root.getMemberNames();
        bool ret = true;
        for (auto it = mems.begin(); it != mems.end(); ++it) {
            if (!Unmarshal(obj[*it], root[*it]))
                ret = false;
        }
#else
        if (!root.is_object())
            return false;
        obj.clear();
        // auto mems = root.getMemberNames();
        bool ret = true;
        for (auto it = root.begin(); it != root.end(); ++it) {
            if (!Unmarshal(obj[*it], root[*it]))
                ret = false;
        }
#endif
        return ret;
    }

    // map key:int
    template <typename T>
    bool Unmarshal(map<int, T>& obj, const nlohmann::json& root) {
#if 0
        if (!root.isObject())
            return false;
        obj.clear();
        auto mems = root.getMemberNames();
        bool ret = true;
        for (auto it = mems.begin(); it != mems.end(); ++it) {
            if (!Unmarshal(obj[atoi(it->c_str())], root[*it]))
                ret = false;
        }
#else
        if (!root.is_object())
            return false;
        obj.clear();
        //auto mems = root.getMemberNames();
        bool ret = true;
        for (auto it = root.begin(); it != root.end(); ++it) {
            if (!Unmarshal(obj[atoi(it.key().c_str())], root[*it]))
                ret = false;
        }
#endif
        return ret;
    }

    // map key:long
    template <typename T>
    bool Unmarshal(map<long, T>& obj, const nlohmann::json& root) {
        // if (!root.isObject())
        if (!root.is_object())
            return false;
        obj.clear();
#if 0
        auto mems = root.getMemberNames();
        bool ret = true;
        for (auto it = mems.begin(); it != mems.end(); ++it) {
            if (!Unmarshal(obj[atol(it->c_str())], root[*it]))
                ret = false;
        }
#else
        bool ret = true;
        for (auto it = root.begin(); it != root.end(); ++it) {
            if (!Unmarshal(obj[atol(it.key().c_str())], root[*it]))
                ret = false;
        }
#endif
        return ret;
    }

    /*======  Marshalʵ��  =======*/

    // ���һ���� �Ƿ��зǾ�̬�����ص�marshal����
    template <typename T>
    struct TestMarshalFunc {
        template <typename TT>
        static char func(decltype(&TT::marshal));
        template <typename TT>
        static int func(...);
        const static bool has = (sizeof(func<T>(NULL)) == sizeof(char));
    };

    template <typename T, typename enable_if<TestMarshalFunc<T>::has, int>::type = 0>
    inline void Marshal(const T& obj, nlohmann::json& root) {
        obj.marshal(root);
    }
    template <typename T, typename enable_if<!TestMarshalFunc<T>::has, int>::type = 0>
    inline void Marshal(const T& obj, nlohmann::json& root) {
        root = obj;
    }

    //������jsoncpp ��֧�ֵĻ�������
    template <int = 0>
    inline void Marshal(long obj, nlohmann::json& root) {
        // root = Int64(obj);
        root = int64_t(obj);
    }

    //������jsoncpp ��֧�ֵĻ�������
    template <int = 0>
    inline void Marshal(uint64_t obj, nlohmann::json& root) {
        // root = UInt64(obj);
        root = uint64_t(obj);
    }

    //������������û��ʵ��marshal�������� vector map
    template <typename T>
    void Marshal(const vector<T>&, nlohmann::json&);
    template <typename T>
    void Marshal(const map<string, T>&, nlohmann::json&);
    template <typename T>
    void Marshal(const map<int, T>& obj, nlohmann::json& root);
    template <typename T>
    void Marshal(const map<long, T>& obj, nlohmann::json& root);
    ;
    // vector
    template <typename T>
    void Marshal(const vector<T>& obj, nlohmann::json& root) {
        for (unsigned int i = 0; i < obj.size(); ++i) {
            Marshal(obj[i], root[i]);
        }
    }

    // map key:string
    template <typename T>
    void Marshal(const map<string, T>& obj, nlohmann::json& root) {
        for (auto it = obj.begin(); it != obj.end(); ++it) {
            Marshal(it->second, root[it->first]);
        }
    }

    // map key:int
    template <typename T>
    void Marshal(const map<int, T>& obj, nlohmann::json& root) {
        char num_buf[15] = { 0 };
        for (auto it = obj.begin(); it != obj.end(); ++it) {
            snprintf(num_buf, 15, "%d", it->first);
            Marshal(it->second, root[num_buf]);
        }
    }

    // map key:long
    template <typename T>
    void Marshal(const map<long, T>& obj, nlohmann::json& root) {
        char num_buf[25] = { 0 };
        for (auto it = obj.begin(); it != obj.end(); ++it) {
            snprintf(num_buf, 25, "%ld", it->first);
            Marshal(it->second, root[num_buf]);
        }
    }

    // map key:shared_ptr
    template <typename T>
    void Marshal(std::shared_ptr<T> obj, nlohmann::json& root) {
        Marshal(*obj.get(), root);
    }

    /*========  string �汾  ===========*/
    template <typename T>
    void Marshal(const T& obj, string& s) {
        nlohmann::json root;
        //Json::FastWriter writer;
        Marshal(obj, root);
        if (root.is_null())
            s = "";
        else
            //s = writer.write(root);
            s = root.dump();
    }

    template <typename T>
    bool Unmarshal(T& obj, const string& s) {
        //Json::Reader reader;
        nlohmann::json root;
        root = nlohmann::json::parse(s);
        // if (!reader.parse(s, root))
        //   return false;
        return Unmarshal(obj, root);
    }
}  // namespace Json

// һЩ���õĺ�,Сд���������ĺ�ֻ�ޱ��ļ���ʹ��

// ���������ȫչ��Ϊ�ַ���,������
#define TO_STRING(...) __to_string__(__VA_ARGS__)
#define __to_string__(...) #__VA_ARGS__

// ͳ�ƿɱ���������ĺ�
#define COUNT(...) __count__(0, ##__VA_ARGS__, 21,20, 19, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0)
#define __count__(_0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20,_21, N, \
                  ...)                                                                                              \
    N

// ��������չ����,������һ��
#define MACRO_CAT(a, b) __macro_cat__(a, b)
#define __macro_cat__(a, b) a##b

// ��չ��ʵ��αѭ��
#define __func_1(func, member) func(member);
#define __func_2(func, member, ...) __func_1(func, member) __func_1(func, __VA_ARGS__)
#define __func_3(func, member, ...) __func_1(func, member) __func_2(func, __VA_ARGS__)
#define __func_4(func, member, ...) __func_1(func, member) __func_3(func, __VA_ARGS__)
#define __func_5(func, member, ...) __func_1(func, member) __func_4(func, __VA_ARGS__)
#define __func_6(func, member, ...) __func_1(func, member) __func_5(func, __VA_ARGS__)
#define __func_7(func, member, ...) __func_1(func, member) __func_6(func, __VA_ARGS__)
#define __func_8(func, member, ...) __func_1(func, member) __func_7(func, __VA_ARGS__)
#define __func_9(func, member, ...) __func_1(func, member) __func_8(func, __VA_ARGS__)
#define __func_10(func, member, ...) __func_1(func, member) __func_9(func, __VA_ARGS__)
#define __func_11(func, member, ...) __func_1(func, member) __func_10(func, __VA_ARGS__)
#define __func_12(func, member, ...) __func_1(func, member) __func_11(func, __VA_ARGS__)
#define __func_13(func, member, ...) __func_1(func, member) __func_12(func, __VA_ARGS__)
#define __func_14(func, member, ...) __func_1(func, member) __func_13(func, __VA_ARGS__)
#define __func_15(func, member, ...) __func_1(func, member) __func_14(func, __VA_ARGS__)
#define __func_16(func, member, ...) __func_1(func, member) __func_15(func, __VA_ARGS__)
#define __func_17(func, member, ...) __func_1(func, member) __func_16(func, __VA_ARGS__)
#define __func_18(func, member, ...) __func_1(func, member) __func_17(func, __VA_ARGS__)
#define __func_19(func, member, ...) __func_1(func, member) __func_18(func, __VA_ARGS__)
#define __func_20(func, member, ...) __func_1(func, member) __func_19(func, __VA_ARGS__)
#define __func_21(func, member, ...) __func_1(func, member) __func_20(func, __VA_ARGS__)
#define __func_22(func, member, ...) __func_1(func, member) __func_21(func, __VA_ARGS__)
#define __func_23(func, member, ...) __func_1(func, member) __func_22(func, __VA_ARGS__)
// ����һ������func��һ��list����func����list��ÿ��Ԫ����
#define FOR_EACH(func, ...) MACRO_CAT(__func_, COUNT(__VA_ARGS__))(func, __VA_ARGS__)

//��һ�������ڲ����� unmarshal ����.
//��������ÿ��field,������֤ȫ����ȷ,δ����ȷ������field����ԭ��ֵ����. ����field ������ȷ����ʱ����true
#define UNMARSHAL_OBJ(...)                                  \
    bool unmarshal(const nlohmann::json& root) {               \
        std::string uni_err_message = "";\
        bool ret = true;                                    \
        FOR_EACH(__unmarshal_obj_each_field__, __VA_ARGS__) \
        if (uni_err_message != "") { \
          std::cout << "[WARN] " << uni_err_message << std::endl; }\
          return ret;                                         \
    }
#define __unmarshal_obj_each_field__(field)      \
    if (root.contains(#field)) {\
    if (!Json::Unmarshal(field, root[#field])) { \
        ret = false;                             \
    }} else {\
        uni_err_message = "NOTICE!!! field:[" + std::string(#field) + "] not matched, will use default value.";\
    }

//��һ�������ڲ����� marshal ����
#define MARSHAL_OBJ(...) \
    void marshal(nlohmann::json& root) const { FOR_EACH(__marshal_obj_each_field__, __VA_ARGS__) }
#define __marshal_obj_each_field__(field) Json::Marshal(field, root[#field]);

//��һ��������Ա� unmarshal��marshal
#define JSON_HELP(...)         \
    UNMARSHAL_OBJ(__VA_ARGS__) \
    MARSHAL_OBJ(__VA_ARGS__)

#endif // CPPBATIS_EASY_JSON_HPP
