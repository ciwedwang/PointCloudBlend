/*
 *  ��������: ������һЩ���ƹ���ͨ�õ�����
 */
namespace cloud_icp_reg {
#ifndef DEG2RAD
#define DEG2RAD(x) ((x)*0.017453293)
#endif

#ifndef RAD2DEG
#define RAD2DEG(x) ((x)*57.29578)
#endif

typedef pcl::PointXYZRGBA CloudItem;//XYZRGBA��ʽ�ĵ��Ƶ�Ԫ
typedef pcl::PointCloud< CloudItem> Cloud;//��XYZRGBA��ʽ�ĵ��Ƶ�Ԫ���ɵĵ�������
typedef Cloud::ConstPtr CloudConstPtr;//ָ�������ɫ��Ϣ�ĵ��Ƶĳ���ָ��
typedef Cloud::Ptr CloudPtr;//ָ�������ɫ��Ϣ�ĵ��Ƶ�ָ��,����pcl��PtrΪshared_ptr����

typedef pcl::PointXYZI CloudIItem;//XYZI��ʽ�ĵ��Ƶ�Ԫ,����IΪǿ��
typedef pcl::PointCloud< CloudIItem> CloudI;//��XYZI��ʽ�ĵ��Ƶ�Ԫ���ɵĵ�������
typedef CloudI::ConstPtr CloudIConstPtr;//ָ�����ǿ����Ϣ�ĵ��Ƶĳ���ָ��
typedef CloudI::Ptr CloudIPtr;//ָ�����ǿ����Ϣ�ĵ��Ƶ�ָ��

//����Ĭ�ϵĿ����͸��ƹ��캯��
#define DISALLOW_COPY_AND_ASSIGN(TypeName) \
            TypeName(const TypeName&); \
            TypeName& operator=(const TypeName&)

//����ģʽ
#define SINGLETON_CLASS(class_name)\
    private:\
    class_name();\
    public:\
    static class_name & instance()\
        {\
        static class_name ins;\
        return ins;\
        }
} // cloud_icp_reg