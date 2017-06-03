#include<string>
#include<windows.h>
#include<iostream>
#include<fstream>
#include<Kinect.h>
#include <NuiKinectFusionApi.h>
#include<opencv2/opencv.hpp>

using namespace std;
using namespace cv;

#ifndef SAFE_FUSION_RELEASE_IMAGE_FRAME
#define SAFE_FUSION_RELEASE_IMAGE_FRAME(p) { if (p) { static_cast<void>(NuiFusionReleaseImageFrame(p)); (p)=NULL; } }
#endif
#ifndef SAFE_DELETE_ARRAY
#define SAFE_DELETE_ARRAY(p) { if (p) { delete[] (p); (p)=NULL; } }
#endif

// Safe release for interfaces
template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}
/// Set Identity in a Matrix4
void SetIdentityMatrix(Matrix4 &mat)
{
	mat.M11 = 1; mat.M12 = 0; mat.M13 = 0; mat.M14 = 0;
	mat.M21 = 0; mat.M22 = 1; mat.M23 = 0; mat.M24 = 0;
	mat.M31 = 0; mat.M32 = 0; mat.M33 = 1; mat.M34 = 0;
	mat.M41 = 0; mat.M42 = 0; mat.M43 = 0; mat.M44 = 1;
}

void UpdateIntrinsics(NUI_FUSION_IMAGE_FRAME * pImageFrame, NUI_FUSION_CAMERA_PARAMETERS * params)
{
	if (pImageFrame != nullptr && pImageFrame->pCameraParameters != nullptr && params != nullptr)
	{
		pImageFrame->pCameraParameters->focalLengthX = params->focalLengthX;
		pImageFrame->pCameraParameters->focalLengthY = params->focalLengthY;
		pImageFrame->pCameraParameters->principalPointX = params->principalPointX;
		pImageFrame->pCameraParameters->principalPointY = params->principalPointY;
	}

	// Confirm we are called correctly
	_ASSERT(pImageFrame != nullptr && pImageFrame->pCameraParameters != nullptr && params != nullptr);
}

//
//class Kinect
//{
//public:
//	static const int cColorWidth = 1920;
//	static const int cColorHeight = 1080;
//	Kinect();
//	~Kinect();
//	HRESULT InitKinect();
//	void Update();
//	void ProcessColor(RGBQUAD* pBuffer, int nWidth, int nHeight);
//private:
//	IKinectSensor* m_pKinectSensor;
//	IColorFrameReader* m_pColorFrameReader;
//	RGBQUAD* m_pColorRGBX;
//
//};
//Kinect::Kinect() {
//	m_pKinectSensor = NULL;
//	m_pColorFrameReader = NULL;
//	m_pColorRGBX = new RGBQUAD[cColorWidth * cColorHeight];
//}
//Kinect::~Kinect() {
//	if (m_pColorRGBX){
//		delete[] m_pColorRGBX;
//		m_pColorRGBX = NULL;
//	}
//	SafeRelease(m_pColorFrameReader);
//	if (m_pKinectSensor) {
//		m_pKinectSensor->Close();
//	}
//	SafeRelease(m_pKinectSensor);
//}
//HRESULT Kinect::InitKinect() {
//	HRESULT hr;
//	hr = GetDefaultKinectSensor(&m_pKinectSensor);
//	if (FAILED(hr)) {
//		cout << "cann't get kinect sensor!" << endl;
//		return hr;
//	}
//	if (m_pKinectSensor) {
//		IColorFrameSource* pColorFrameSource = NULL;
//		hr = m_pKinectSensor->Open();
//		if (SUCCEEDED(hr))
//			hr = m_pKinectSensor->get_ColorFrameSource(&pColorFrameSource);
//		if (SUCCEEDED(hr))
//			hr = pColorFrameSource->OpenReader(&m_pColorFrameReader);
//		SafeRelease(pColorFrameSource);
//	}
//	if (!m_pKinectSensor || FAILED(hr)) {
//		cout << "No ready Kinect ready!" << endl;
//		return E_FAIL;
//	}
//	return hr;
//}
//void Kinect::Update() {
//	if (!m_pColorFrameReader)
//		return;
//	IColorFrame* pColorFrame = NULL;
//	HRESULT hr = m_pColorFrameReader->AcquireLatestFrame(&pColorFrame);
//	if (SUCCEEDED(hr)) {
//		IFrameDescription* pFrameDescription = NULL;
//		int nWidth = 0;
//		int nHeight = 0;
//		ColorImageFormat imageFormat = ColorImageFormat_None;
//		UINT nBufferSize = NULL;
//		RGBQUAD* pBuffer = NULL;
//		if (SUCCEEDED(hr)){
//			hr = pColorFrame->get_FrameDescription(&pFrameDescription);
//		}
//		if (SUCCEEDED(hr)) {
//			hr = pFrameDescription->get_Width(&nWidth);
//		}
//		if (SUCCEEDED(hr)){
//			hr = pFrameDescription->get_Height(&nHeight);
//		}
//		if (SUCCEEDED(hr)) {
//			hr = pColorFrame->get_RawColorImageFormat(&imageFormat);
//		}
//		if (SUCCEEDED(hr)) {
//			if (imageFormat == ColorImageFormat_Bgra) {
//				hr = pColorFrame->AccessRawUnderlyingBuffer(&nBufferSize, reinterpret_cast<BYTE**>(&pBuffer));
//			}
//			else if (m_pColorRGBX) {
//				pBuffer = m_pColorRGBX;
//				nBufferSize = cColorWidth * cColorHeight * sizeof(RGBQUAD);
//				hr = pColorFrame->CopyConvertedFrameDataToArray(nBufferSize, reinterpret_cast<BYTE*>(pBuffer), ColorImageFormat_Bgra);
//			}
//			else {
//				hr = E_FAIL;
//			}
//		}
//		if (SUCCEEDED(hr)) {
//			ProcessColor(pBuffer, nWidth, nHeight);
//		}
//		SafeRelease(pFrameDescription);
//	}
//	SafeRelease(pColorFrame);
//}
//void Kinect::ProcessColor(RGBQUAD* pBuffer, int nWidth, int nHeight) {
//	if (pBuffer && (nWidth == cColorWidth) && (nHeight == cColorHeight)) {
//		Mat img(nHeight, nWidth, CV_8UC4, pBuffer);
//		Mat showImg;
//		resize(img, showImg, Size(nWidth / 2, nHeight / 2));
//		imshow("image", showImg);
//		waitKey(1000);
//	}
//}
//int main() {
//	Kinect kinect;
//	kinect.InitKinect();
//	//while (1) {
//		kinect.Update();		
//		getchar();
//		//waitKey(1000);
//		kinect.Update();
//	//	if (waitKey(1000) >= 0) {
//	//		break;
//	//	}
//
//	//}
//	return 0;
//}

class KinectFusion
{
public:
	KinectFusion();
	HRESULT CreateFirstConnected();
	HRESULT InitializeKinectFusion();
	HRESULT ResetReconstruction();
	HRESULT SetupUndistortion();
	HRESULT OnCoordinateMappingChanged();
	HRESULT MapColorToDepth();
	void update();
	void processDepth();
	void precessDepthToColor();
	void processDepthToCloud();
	void mProcessDepth();
	void processResult();
	void getPointsCloud();
	void getColorDepthSize();
	~KinectFusion();
	int                         m_cFrameCounter;		//camera tracking���Ѿ��е�frame�ĸ���
	bool						m_bTrackingFailed;		//camera tracking�Ƿ�ʧ�ܱ�־
	int							m_cLostFrameCounter;	//����֮֡�䶪ʧ��֡��
private:

	IKinectSensor*              m_pNuiSensor;			// Current Kinect	
	IMultiSourceFrameReader*	m_pMultiFrameReader;	
	IColorFrameReader*			m_pColorFrameReader;	//Color reader	
	IDepthFrameReader*          m_pDepthFrameReader;	// Depth reader
	ICoordinateMapper*          m_pMapper;				//����ϵת��
	WAITABLE_HANDLE             m_coordinateMappingChangedEvent;//����ϵӳ��任��־

	static const UINT                         m_cDepthWidth = 512;
	static const UINT                         m_cDepthHeight = 424;
	static const UINT                         m_cDepthImagePixels = 512 * 424;
	static const UINT						  m_cColorWidth = 1920;
	static const UINT						  m_cColorHeight = 1080;
	static const int						  cBytesPerPixel = 4; // for depth float and int-per-pixel raycast images
	static const int						  cVisibilityTestQuantShift = 2; // shift by 2 == divide by 4
	static const UINT16						  cDepthVisibilityTestThreshold = 50; //50 mm

	Mat i_rgb;      //ע�⣺�������Ϊ4ͨ����ͼ��Kinect������ֻ����Bgra��ʽ����
	UINT16*									 depthData;	//ԭʼ�����ͼ��ֵ

	Mat i_depth;	//������ʾ�����ͼ��ȡ���ͼ16λ�ĺ�8λ
	Mat i_depthToRgb;	//��Ӧ���ͼ�Ĳ�ɫͼ����С�����ͼһ����λ�ú����ͼһһ��Ӧ��


	INuiFusionColorReconstruction*			 m_pVolume;							//Volume
	NUI_FUSION_CAMERA_PARAMETERS			 m_cameraParameters;				//camera���� focalx,focaly, principalPointX, principalPointsY;
	Matrix4									 m_worldToCameraTransform;			//global��camera����ϵ��ת��
	Matrix4									 m_defaultWorldToVolumeTransform;	//Ĭ�ϵ�global��camera����ϵ��ת������	
	NUI_FUSION_RECONSTRUCTION_PARAMETERS	 m_reconstructionParams;	//volume �Ĳ���

	float									 m_fMinDepthThreshold;		//�Ի�õ�depthͼ��ͨ����������ֵ���е�һ�����ˣ���λΪ��
	float									 m_fMaxDepthThreshold;
	unsigned short							 m_cMaxIntegrationWeight;	//�����ͼ���Ͻ�global modelʱ�õ�����ʱƽ������,ԽС������Խ�࣬�ʺ϶�̬��Խ���ں�Խ����ϸ�ڸ��࣬��������١�
	int										 m_deviceIndex;				//��ʹ��GPUʱ��ѡ����豸������
	NUI_FUSION_RECONSTRUCTION_PROCESSOR_TYPE m_processorType;			//ʹ��GPU��CPU

	ColorSpacePoint*			m_pColorCoordinates;		//��ɫ����
	CameraSpacePoint*			m_pCameraCoordinates;		//3D�ռ�����꣨�������ϵ��ܣ�
	DepthSpacePoint*			m_pDepthDistortionMap;	//���ͼ���꣨ÿ��ֵ�������ͼ�ϵ����꣩
	UINT*                       m_pDepthDistortionLT;		////���camera frame�µ������Ƿ����ͶӰ�����ͼ�ϣ������У��⽫��Ӧλ�ã��˴���һά��ʾ����ֵ����Ϊ�ܴ������ԣ�����Ӧλ��ֵ��ֵΪλ������ֵ�����һά�������ڹ���ԭʼ���ͼ
	UINT16*                     m_pDepthImagePixelBuffer;	//����ԭʼ�����ͼ����m_pDepthDistortionLT���˺�����ֵ������ȡ�ĵط���ֵΪ0
	UINT16*                     m_pDepthVisibilityTestMap;

	NUI_FUSION_IMAGE_FRAME*     m_pDepthFloatImage;			//��m_pDepthImagePixelBuffer��m_fMinDepthThreshold��m_fMaxDepthThreshold���˺��������ݡ�
	NUI_FUSION_IMAGE_FRAME*		m_pSmoothDepthFloatImage;	//��m_pDepthFloatImageƽ������������֡
	NUI_FUSION_IMAGE_FRAME*     m_pPointCloud;				//��i֡����ͶӰ�����(��Ϊ��֡����Ƚ��)���ں���һ֡����tracking
	NUI_FUSION_IMAGE_FRAME*     m_pShadedSurface;			//���ǻ����ģ��
	NUI_FUSION_IMAGE_FRAME*     m_pColorImage;				//color����
	NUI_FUSION_IMAGE_FRAME*		m_pResampledColorImageDepthAligned;     //���ͼ��Ӧ�Ĳ�ɫֵ
	NUI_FUSION_IMAGE_FRAME*		m_pCapturedSurfaceColor;	////��i֡����ͶӰ�ĵ��Ƶ���ɫ

	bool						m_bHaveValidCameraParameters;	//�Ƿ��кϷ����������
	bool                        m_bInitializeError;				//��ʼ���Ƿ�ɹ���־
	bool                        m_bMirrorDepthFrame;			//�Ƿ������Ⱦ����־
	bool						m_bTranslateResetPoseByMinDepthThreshold;	//�Ƿ�volume��z��������world frame����ϵ��ƽ��
};

KinectFusion::KinectFusion() {
	m_pMultiFrameReader = NULL;
	i_rgb.create(m_cColorHeight, m_cColorWidth, CV_8UC4);
	i_depth.create(m_cDepthHeight, m_cDepthWidth, CV_8UC1);
	i_depthToRgb.create(m_cDepthHeight, m_cDepthWidth, CV_8UC4);
	m_pNuiSensor = NULL;
	m_pColorFrameReader = NULL;
	m_pDepthFrameReader = NULL;
	depthData = new UINT16[m_cDepthImagePixels];
	m_pMapper = NULL;
	m_coordinateMappingChangedEvent = NULL;
	m_pColorCoordinates = new ColorSpacePoint[m_cDepthImagePixels];
	m_pCameraCoordinates = new CameraSpacePoint[m_cDepthImagePixels];
	// Define a cubic Kinect Fusion reconstruction volume,
	// with the Kinect at the center of the front face and the volume directly in front of Kinect.
	m_reconstructionParams.voxelsPerMeter = 256;// 1000mm / 256vpm = ~3.9mm/voxel    
	m_reconstructionParams.voxelCountX = 384;   // 384 / 256vpm = 1.5m wide reconstruction
	m_reconstructionParams.voxelCountY = 384;   // Memory = 384*384*384 * 4bytes per voxel
	m_reconstructionParams.voxelCountZ = 384;   // This will require a GPU with at least 256MB

	m_fMinDepthThreshold = NUI_FUSION_DEFAULT_MINIMUM_DEPTH;   // min depth in meters
	m_fMaxDepthThreshold = NUI_FUSION_DEFAULT_MAXIMUM_DEPTH;    // max depth in meters
	// This parameter is the temporal averaging parameter for depth integration into the reconstruction
	m_cMaxIntegrationWeight = NUI_FUSION_DEFAULT_INTEGRATION_WEIGHT;	// Reasonable for static scenes
	m_deviceIndex = -1;	//�Զ�ѡ��GPU�豸
	m_processorType = NUI_FUSION_RECONSTRUCTION_PROCESSOR_TYPE_AMP; //ʹ��GPU��CPU

	SetIdentityMatrix(m_worldToCameraTransform);
	SetIdentityMatrix(m_defaultWorldToVolumeTransform);

	// We don't know these at object creation time, so we use nominal values.
	// These will later be updated in response to the CoordinateMappingChanged event.
	m_cameraParameters.focalLengthX = NUI_KINECT_DEPTH_NORM_FOCAL_LENGTH_X;		//���ֵ�ڿ�ʼ�ǲ�֪���ģ����ں����ĵõ�����Ҫ����
	m_cameraParameters.focalLengthY = NUI_KINECT_DEPTH_NORM_FOCAL_LENGTH_Y;
	m_cameraParameters.principalPointX = NUI_KINECT_DEPTH_NORM_PRINCIPAL_POINT_X;
	m_cameraParameters.principalPointY = NUI_KINECT_DEPTH_NORM_PRINCIPAL_POINT_Y;
	m_pVolume = NULL;
	m_pDepthFloatImage = nullptr;
	m_pSmoothDepthFloatImage = nullptr;
	m_pDepthImagePixelBuffer = nullptr;
	m_pPointCloud = nullptr;
	m_pShadedSurface = nullptr;
	m_pDepthDistortionMap = nullptr;
	m_pDepthDistortionLT = nullptr;
	m_bHaveValidCameraParameters = false;
	m_bInitializeError = false;
	m_bMirrorDepthFrame = false;
	m_bTrackingFailed = false;
	m_cFrameCounter = 0;
	m_cLostFrameCounter = 0;
	m_bTranslateResetPoseByMinDepthThreshold = true;
	m_pColorImage = nullptr;
	m_pResampledColorImageDepthAligned = nullptr;
	m_pCapturedSurfaceColor = nullptr;
	m_pDepthVisibilityTestMap = nullptr;

}
KinectFusion::~KinectFusion() {	
	SafeRelease(m_pColorFrameReader);
	SafeRelease(m_pMultiFrameReader);
	SafeRelease(m_pDepthFrameReader);
	SafeRelease(m_pMapper);
	if (nullptr != m_pMapper)
		m_pMapper->UnsubscribeCoordinateMappingChanged(m_coordinateMappingChangedEvent);
	if (m_pNuiSensor) {
		m_pNuiSensor->Close();
	}
	SafeRelease(m_pNuiSensor);
	SafeRelease(m_pVolume);
	SAFE_DELETE_ARRAY(depthData);
	SAFE_DELETE_ARRAY(m_pColorCoordinates);
	SAFE_DELETE_ARRAY(m_pCameraCoordinates);
	SAFE_DELETE_ARRAY(m_pDepthDistortionMap);
	SAFE_DELETE_ARRAY(m_pDepthDistortionLT);
	SAFE_DELETE_ARRAY(m_pDepthImagePixelBuffer);
	SAFE_DELETE_ARRAY(m_pDepthVisibilityTestMap);
	SAFE_FUSION_RELEASE_IMAGE_FRAME(m_pDepthFloatImage);
	SAFE_FUSION_RELEASE_IMAGE_FRAME(m_pSmoothDepthFloatImage);
	SAFE_FUSION_RELEASE_IMAGE_FRAME(m_pPointCloud);
	SAFE_FUSION_RELEASE_IMAGE_FRAME(m_pShadedSurface);
	SAFE_FUSION_RELEASE_IMAGE_FRAME(m_pColorImage);
	SAFE_FUSION_RELEASE_IMAGE_FRAME(m_pResampledColorImageDepthAligned);
	SAFE_FUSION_RELEASE_IMAGE_FRAME(m_pCapturedSurfaceColor);

}
HRESULT KinectFusion::CreateFirstConnected()
{
	HRESULT hr;

	hr = GetDefaultKinectSensor(&m_pNuiSensor);
	if (FAILED(hr))
	{
		return hr;
	}

	//if (m_pNuiSensor)
	//{
	//	//Initialize the Kinect
	//	hr = m_pNuiSensor->Open();
	//	//get color reader
	//	IColorFrameSource* pColorFrameSource = NULL;		
	//	if (SUCCEEDED(hr))
	//		hr = m_pNuiSensor->get_ColorFrameSource(&pColorFrameSource);
	//	if (SUCCEEDED(hr))
	//		hr = pColorFrameSource->OpenReader(&m_pColorFrameReader);
	//	SafeRelease(pColorFrameSource);
	//	// get the depth reader
	//	IDepthFrameSource* pDepthFrameSource = NULL;
	//	if (SUCCEEDED(hr))
	//	{
	//		hr = m_pNuiSensor->get_DepthFrameSource(&pDepthFrameSource);
	//	}
	//	if (SUCCEEDED(hr))
	//	{
	//		hr = m_pNuiSensor->get_CoordinateMapper(&m_pMapper);
	//	}
	//	if (SUCCEEDED(hr))
	//	{
	//		hr = m_pMapper->SubscribeCoordinateMappingChanged(&m_coordinateMappingChangedEvent);
	//	}
	//	if (SUCCEEDED(hr))
	//	{
	//		hr = pDepthFrameSource->OpenReader(&m_pDepthFrameReader);
	//	}
	//	if (SUCCEEDED(hr)) {
	//		hr = m_pNuiSensor->OpenMultiSourceFrameReader(FrameSourceTypes::FrameSourceTypes_Color |
	//			FrameSourceTypes::FrameSourceTypes_Depth, &m_pMultiFrameReader);
	//	}
	//	SafeRelease(pDepthFrameSource);
	//}

	if (m_pNuiSensor) {
		hr = m_pNuiSensor->Open();
		if (SUCCEEDED(hr)) {
			hr = m_pNuiSensor->OpenMultiSourceFrameReader(FrameSourceTypes::FrameSourceTypes_Color |
				FrameSourceTypes::FrameSourceTypes_Depth, &m_pMultiFrameReader);
		}
		if (SUCCEEDED(hr))
			hr = m_pNuiSensor->get_CoordinateMapper(&m_pMapper);
		if (SUCCEEDED(hr))
			hr = m_pMapper->SubscribeCoordinateMappingChanged(&m_coordinateMappingChangedEvent);
	}
	if (nullptr == m_pNuiSensor || FAILED(hr))
	{
		cout << "No ready Kinect found!" << endl;
		return E_FAIL;
	}
	return hr;
}
HRESULT KinectFusion::InitializeKinectFusion() {
	HRESULT hr = S_OK;
	//�豸���
	// Check to ensure suitable DirectX11 compatible hardware exists before initializing Kinect Fusion
	WCHAR description[MAX_PATH];    //The description of the device.
	WCHAR instancePath[MAX_PATH];	//The DirectX instance path of the GPU being used for reconstruction.
	UINT memorySize = 0;
	if (FAILED(hr = NuiFusionGetDeviceInfo(m_processorType,	m_deviceIndex,	&description[0], ARRAYSIZE(description),	&instancePath[0],	ARRAYSIZE(instancePath),	&memorySize)))
	{
		if (hr == E_NUI_BADINDEX)
		{
			// This error code is returned either when the device index is out of range for the processor 
			// type or there is no DirectX11 capable device installed. As we set -1 (auto-select default) 
			// for the device index in the parameters, this indicates that there is no DirectX11 capable 
			// device. The options for users in this case are to either install a DirectX11 capable device
			// (see documentation for recommended GPUs) or to switch to non-real-time CPU based 
			// reconstruction by changing the processor type to NUI_FUSION_RECONSTRUCTION_PROCESSOR_TYPE_CPU.
			cout << "No DirectX11 device detected, or invalid device index - Kinect Fusion requires a DirectX11 device for GPU-based reconstruction." << endl;
		}
		else
			cout << "Failed in call to NuiFusionGetDeviceInfo." << endl;
		return hr;
	}
	//����Fusion �ݻ��ؽ� Volume
	hr = NuiFusionCreateColorReconstruction(&m_reconstructionParams, m_processorType, m_deviceIndex, &m_worldToCameraTransform, &m_pVolume);

	if (FAILED(hr))
	{
		if (E_NUI_GPU_FAIL == hr)
			cout<<"Device "<<m_deviceIndex<<" not able to run Kinect Fusion, or error initializing."<<endl;
		else if (E_NUI_GPU_OUTOFMEMORY == hr)			
			cout<<"Device "<<m_deviceIndex<<" out of memory error initializing reconstruction - try a smaller reconstruction volume."<<endl;
		else if (NUI_FUSION_RECONSTRUCTION_PROCESSOR_TYPE_CPU != m_processorType)
			cout<<"Failed to initialize Kinect Fusion reconstruction volume on device"<<m_deviceIndex<<endl;
		else
			cout<<"Failed to initialize Kinect Fusion reconstruction volume on CPU."<<endl;
		return hr;
	}

	//���洴����volume��WorldToVolumeTransform����
	hr = m_pVolume->GetCurrentWorldToVolumeTransform(&m_defaultWorldToVolumeTransform);
	if (FAILED(hr))
	{
		cout<<"Failed in call to GetCurrentWorldToVolumeTransform."<<endl;
		return hr;
	}
	//�Ƿ�volume��z�ᣨ����world frame����ϵ����������ƽ��һ��voxels(��Ϊkinect��С��depthֵҲ�����0.5m���ң��ʾ�����volume���ؽ���һ���֣�Ҳû��ģ��)
	if (m_bTranslateResetPoseByMinDepthThreshold)
	{
		//���¹���volume
		hr = ResetReconstruction();
		if (FAILED(hr))
		{
			return hr;
		}
	}

	// �����������֡
	hr = NuiFusionCreateImageFrame(NUI_FUSION_IMAGE_TYPE_FLOAT, m_cDepthWidth, m_cDepthHeight, &m_cameraParameters, &m_pDepthFloatImage);
	if (FAILED(hr))
	{
		cout<<"Failed to initialize Kinect Fusion m_pDepthFloatImage."<<endl;
		return hr;
	}
	// ����ƽ���������֡
	hr = NuiFusionCreateImageFrame(NUI_FUSION_IMAGE_TYPE_FLOAT, m_cDepthWidth, m_cDepthHeight, nullptr, &m_pSmoothDepthFloatImage);
	if (FAILED(hr))
	{
		cout << "Failed to initialize Kinect Fusion m_pSmoothDepthFloatImage." << endl;
		return hr;
	}
	// ��������ͶӰ����֡
	hr = NuiFusionCreateImageFrame(NUI_FUSION_IMAGE_TYPE_POINT_CLOUD, m_cDepthWidth, m_cDepthHeight, &m_cameraParameters, &m_pPointCloud);
	if (FAILED(hr))
	{
		cout<<"Failed to initialize Kinect Fusion m_pPointCloud."<<endl;
		return hr;
	}
	// ���������ɫ֡
	hr = NuiFusionCreateImageFrame(NUI_FUSION_IMAGE_TYPE_COLOR, m_cDepthWidth, m_cDepthHeight, &m_cameraParameters, &m_pShadedSurface);
	if (FAILED(hr))
	{
		cout<<"Failed to initialize Kinect Fusion m_pShadedSurface."<<endl;
		return hr;
	}
	// ������ɫ֡
	hr = NuiFusionCreateImageFrame(NUI_FUSION_IMAGE_TYPE_COLOR, m_cColorWidth, m_cColorHeight, &m_cameraParameters, &m_pColorImage);
	if (FAILED(hr))
	{
		cout << "Failed to initialize Kinect Fusion m_pShadedSurface." << endl;
		return hr;
	}
	// �������ͼ��С�Ĳ�ɫͼ֡
	if (FAILED(hr = NuiFusionCreateImageFrame(NUI_FUSION_IMAGE_TYPE_COLOR, m_cDepthWidth, m_cDepthHeight, &m_cameraParameters, &m_pResampledColorImageDepthAligned)))
	{
		cout << "Failed to initialize Kinect Fusion m_pShadedSurface"<<endl;
		return hr;
	}
	//��������ͶӰ����֡��ɫ
	if (FAILED(hr = NuiFusionCreateImageFrame(NUI_FUSION_IMAGE_TYPE_COLOR, m_cDepthWidth, m_cDepthHeight, &m_cameraParameters, &m_pCapturedSurfaceColor)))
	{
		return hr;
	}

	//�������ͼ��ص��ڴ�
	_ASSERT(m_pDepthImagePixelBuffer == nullptr);
	m_pDepthImagePixelBuffer = new(std::nothrow) UINT16[m_cDepthImagePixels];
	if (nullptr == m_pDepthImagePixelBuffer)
	{
		cout<<"Failed to initialize Kinect Fusion depth image pixel buffer."<<endl;
		return hr;
	}
	_ASSERT(m_pDepthDistortionMap == nullptr);
	m_pDepthDistortionMap = new(std::nothrow) DepthSpacePoint[m_cDepthImagePixels];
	if (nullptr == m_pDepthDistortionMap)
	{
		cout<<"Failed to initialize Kinect Fusion depth image distortion buffer."<<endl;
		return E_OUTOFMEMORY;
	}
	SAFE_DELETE_ARRAY(m_pDepthDistortionLT);
	m_pDepthDistortionLT = new(std::nothrow) UINT[m_cDepthImagePixels];
	if (nullptr == m_pDepthDistortionLT)
	{
		cout<<"Failed to initialize Kinect Fusion depth image distortion Lookup Table."<<endl;
		return E_OUTOFMEMORY;
	}


	SAFE_DELETE_ARRAY(m_pDepthVisibilityTestMap);
	m_pDepthVisibilityTestMap = new(std::nothrow) UINT16[(m_cColorWidth >> cVisibilityTestQuantShift) * (m_cColorHeight >> cVisibilityTestQuantShift)];

	if (nullptr == m_pDepthVisibilityTestMap)
	{
		cout << "Failed to initialize Kinect Fusion depth points visibility test buffer." << endl;
		return E_OUTOFMEMORY;
	}


	// If we have valid parameters, let's go ahead and use them.
	if (m_cameraParameters.focalLengthX != 0)
		SetupUndistortion();
	return hr;
}
HRESULT KinectFusion::ResetReconstruction()
{
	if (nullptr == m_pVolume)
		return E_FAIL;
	HRESULT hr = S_OK;
	SetIdentityMatrix(m_worldToCameraTransform);

	//��volume��Z������������ƽ����С��ȴ�С��voxels������������С��ȵ�world points������volume���z���ӦΪ0.��volume�������һ����������ֻ�Դ������ڵĵ��ƽ��д���
	if (m_bTranslateResetPoseByMinDepthThreshold)
	{
		Matrix4 worldToVolumeTransform = m_defaultWorldToVolumeTransform;

		// ��volume��Z����ƽ����С��ȴ�С��voxels(��0����С����ڲ������ؽ���ģ�ͣ��ʲ���Ҫ����Щ���ִ���volume)
		float minDist = (m_fMinDepthThreshold < m_fMaxDepthThreshold) ? m_fMinDepthThreshold : m_fMaxDepthThreshold;
		worldToVolumeTransform.M43 -= (minDist * m_reconstructionParams.voxelsPerMeter);
		hr = m_pVolume->ResetReconstruction(&m_worldToCameraTransform, &worldToVolumeTransform);
	}
	else
		hr = m_pVolume->ResetReconstruction(&m_worldToCameraTransform, nullptr);
	m_cLostFrameCounter = 0;
	m_cFrameCounter = 0;
	if (SUCCEEDED(hr))
	{
		m_bTrackingFailed = false;
		cout << "Reconstruction has been reset." << endl;
	}
	else
		cout << "Failed to reset reconstruction." << endl;
	return hr;
}
HRESULT KinectFusion::SetupUndistortion()
{
	HRESULT hr = E_UNEXPECTED;

	//���ͼ����ϵԭ�㲻����ͼ�����ģ����������������Ͳ��Ϸ�
	if (m_cameraParameters.principalPointX != 0)
	{
		//���ͼ���ĸ������꣺���ϣ�0��0�������ϣ�1��0������Ϊk����������ֱ𶼳������ͼ�Ŀ�͸ߣ������£�0��1�������£�1��1����ͶӰ��camera frame ����zΪ1��1m���Ŀռ��
		CameraSpacePoint cameraFrameCorners[4] = //at 1 meter distance. Take into account that depth frame is mirrored
		{
			/*LT*/{ -m_cameraParameters.principalPointX / m_cameraParameters.focalLengthX, m_cameraParameters.principalPointY / m_cameraParameters.focalLengthY, 1.f },
			/*RT*/{ (1.f - m_cameraParameters.principalPointX) / m_cameraParameters.focalLengthX, m_cameraParameters.principalPointY / m_cameraParameters.focalLengthY, 1.f },
			/*LB*/{ -m_cameraParameters.principalPointX / m_cameraParameters.focalLengthX, (m_cameraParameters.principalPointY - 1.f) / m_cameraParameters.focalLengthY, 1.f },
			/*RB*/{ (1.f - m_cameraParameters.principalPointX) / m_cameraParameters.focalLengthX, (m_cameraParameters.principalPointY - 1.f) / m_cameraParameters.focalLengthY, 1.f }
		};

		//��4��1m���Ŀռ��߽��ڵĿռ仮��Ϊ���������ͼ��С��ͬ�Ŀռ�㣬Ȼ����Щ��ͶӰ�����ͼ�ϡ�
		for (UINT rowID = 0; rowID < m_cDepthHeight; rowID++)
		{
			const float rowFactor = float(rowID) / float(m_cDepthHeight - 1);
			const CameraSpacePoint rowStart =
			{
				cameraFrameCorners[0].X + (cameraFrameCorners[2].X - cameraFrameCorners[0].X) * rowFactor,
				cameraFrameCorners[0].Y + (cameraFrameCorners[2].Y - cameraFrameCorners[0].Y) * rowFactor,
				1.f
			};

			const CameraSpacePoint rowEnd =
			{
				cameraFrameCorners[1].X + (cameraFrameCorners[3].X - cameraFrameCorners[1].X) * rowFactor,
				cameraFrameCorners[1].Y + (cameraFrameCorners[3].Y - cameraFrameCorners[1].Y) * rowFactor,
				1.f
			};

			const float stepFactor = 1.f / float(m_cDepthWidth - 1);
			const CameraSpacePoint rowDelta =
			{
				(rowEnd.X - rowStart.X) * stepFactor,
				(rowEnd.Y - rowStart.Y) * stepFactor,
				0
			};

			_ASSERT(m_cDepthWidth == NUI_DEPTH_RAW_WIDTH);
			CameraSpacePoint cameraCoordsRow[NUI_DEPTH_RAW_WIDTH];

			CameraSpacePoint currentPoint = rowStart;
			for (UINT i = 0; i < m_cDepthWidth; i++)
			{
				cameraCoordsRow[i] = currentPoint;
				currentPoint.X += rowDelta.X;
				currentPoint.Y += rowDelta.Y;
			}

			hr = m_pMapper->MapCameraPointsToDepthSpace(m_cDepthWidth, cameraCoordsRow, m_cDepthWidth, &m_pDepthDistortionMap[rowID * m_cDepthWidth]);
			if (FAILED(hr))
			{
				cout<<"Failed to initialize Kinect Coordinate Mapper."<<endl;
				return hr;
			}
		}

		if (nullptr == m_pDepthDistortionLT)
		{
			cout<<"Failed to initialize Kinect Fusion depth image distortion Lookup Table."<<endl;
			return E_OUTOFMEMORY;
		}

		//����ͶӰ�ص����ͼλ�ò��Ϸ����⽫�˴�λ�õ����ͼ���Ϊ���ɴӿռ�����ͶӰ���������ں�����˲ɼ��������ͼ
		UINT* pLT = m_pDepthDistortionLT;
		for (UINT i = 0; i < m_cDepthImagePixels; i++, pLT++)
		{
			//nearest neighbor depth lookup table 
			UINT x = UINT(m_pDepthDistortionMap[i].X + 0.5f);
			UINT y = UINT(m_pDepthDistortionMap[i].Y + 0.5f);

			*pLT = (x < m_cDepthWidth && y < m_cDepthHeight) ? x + y * m_cDepthWidth : UINT_MAX;
		}
		m_bHaveValidCameraParameters = true;
	}
	else
	{
		m_bHaveValidCameraParameters = false;
	}
	return S_OK;
}
HRESULT KinectFusion::OnCoordinateMappingChanged()
{
	HRESULT hr = E_UNEXPECTED;

	// Calculate the down sampled image sizes, which are used for the AlignPointClouds calculation frames
	CameraIntrinsics intrinsics = {};

	m_pMapper->GetDepthCameraIntrinsics(&intrinsics);

	float focalLengthX = intrinsics.FocalLengthX / NUI_DEPTH_RAW_WIDTH;
	float focalLengthY = intrinsics.FocalLengthY / NUI_DEPTH_RAW_HEIGHT;
	float principalPointX = intrinsics.PrincipalPointX / NUI_DEPTH_RAW_WIDTH;
	float principalPointY = intrinsics.PrincipalPointY / NUI_DEPTH_RAW_HEIGHT;

	if (m_cameraParameters.focalLengthX == focalLengthX && m_cameraParameters.focalLengthY == focalLengthY &&
		m_cameraParameters.principalPointX == principalPointX && m_cameraParameters.principalPointY == principalPointY)
		return S_OK;

	m_cameraParameters.focalLengthX = focalLengthX;
	m_cameraParameters.focalLengthY = focalLengthY;
	m_cameraParameters.principalPointX = principalPointX;
	m_cameraParameters.principalPointY = principalPointY;

	_ASSERT(m_cameraParameters.focalLengthX != 0);

	UpdateIntrinsics(m_pDepthFloatImage, &m_cameraParameters);
	UpdateIntrinsics(m_pPointCloud, &m_cameraParameters);
	UpdateIntrinsics(m_pShadedSurface, &m_cameraParameters);
	UpdateIntrinsics(m_pSmoothDepthFloatImage, &m_cameraParameters);
	UpdateIntrinsics(m_pColorImage, &m_cameraParameters);
	UpdateIntrinsics(m_pResampledColorImageDepthAligned, &m_cameraParameters);
	UpdateIntrinsics(m_pCapturedSurfaceColor, &m_cameraParameters);

	if (nullptr == m_pDepthDistortionMap)
	{
		cout << "Failed to initialize Kinect Fusion depth image distortion buffer." << endl;
		return E_OUTOFMEMORY;
	}

	hr = SetupUndistortion();
	return hr;
}

void KinectFusion::update() {
	
	if (nullptr == m_pNuiSensor)
	{
		cout << "cannot get kinect sensor!" << endl;
		return;
	}
	//�����������仯
	if (m_coordinateMappingChangedEvent != NULL && WAIT_OBJECT_0 == WaitForSingleObject((HANDLE)m_coordinateMappingChangedEvent, 0))
	{
		cout << "camere corrdinate map chainge!" << endl;
		OnCoordinateMappingChanged();
		ResetEvent((HANDLE)m_coordinateMappingChangedEvent);
	}

	if (nullptr == m_pMultiFrameReader)
	{
		cout << "cannot get multiFrameReader!" << endl;
		return;
	}

	//// ���֡
	//IDepthFrame* pDepthFrame = nullptr;
	//HRESULT hr = m_pDepthFrameReader->AcquireLatestFrame(&pDepthFrame);
	//if (SUCCEEDED(hr)) {
	//	UINT nBufferSize = 0;
	//	//hr = pDepthFrame->CopyFrameDataToArray(nBufferSize, depthData);
	//	hr = pDepthFrame->AccessUnderlyingBuffer(&nBufferSize, &depthData);
	//	
	//	if (SUCCEEDED(hr)) {
	//		hr = m_pMapper->MapDepthFrameToColorSpace(nBufferSize, depthData, nBufferSize, m_pColorCoordinates);			
	//	}
	//	if (SUCCEEDED(hr)) {
	//		precessDepthToColor();
	//	}
	//}
	IDepthFrameReference* m_pDepthFrameReference = NULL;
	IColorFrameReference* m_pColorFrameReference = NULL;
	IDepthFrame* pDepthFrame = NULL;
	IColorFrame* pColorFrame = NULL;
	IMultiSourceFrame* pMultiFrame = nullptr;
	HRESULT hr = m_pMultiFrameReader->AcquireLatestFrame(&pMultiFrame);
	if (SUCCEEDED(hr)) {
		if (SUCCEEDED(hr))
			hr = pMultiFrame->get_ColorFrameReference(&m_pColorFrameReference);
		if (SUCCEEDED(hr))
			hr = m_pColorFrameReference->AcquireFrame(&pColorFrame);
		if (SUCCEEDED(hr))
			hr = pMultiFrame->get_DepthFrameReference(&m_pDepthFrameReference);
		if (SUCCEEDED(hr))
			hr = m_pDepthFrameReference->AcquireFrame(&pDepthFrame);

		//��ȡ�������
		UINT nBufferSize = 0;	//����֡��С
		if (SUCCEEDED(hr))
			hr = pDepthFrame->AccessUnderlyingBuffer(&nBufferSize, &depthData);

		//��ȡ��ɫ����
		NUI_FUSION_BUFFER *destColorBuffer = m_pColorImage->pFrameBuffer;
		if (nullptr == pColorFrame || nullptr == destColorBuffer)
		{
			cout << "create destColorBuffer error!" << endl;
			return ;
		}
		if (SUCCEEDED(hr))				
			hr = pColorFrame->CopyConvertedFrameDataToArray(m_cColorWidth * m_cColorHeight * sizeof(RGBQUAD), destColorBuffer->pBits, ColorImageFormat_Bgra);
			//hr = pColorFrame->CopyConvertedFrameDataToArray(m_cColorWidth * m_cColorHeight * 4, reinterpret_cast<BYTE*>(i_rgb.data), ColorImageFormat::ColorImageFormat_Bgra);
		
		//���������������
		if (SUCCEEDED(hr))
		{
			//copy and remap depth
			const UINT bufferLength = m_cDepthImagePixels;
			UINT16 * pDepth = m_pDepthImagePixelBuffer;
			for (UINT i = 0; i < bufferLength; i++, pDepth++)
			{
				const UINT id = m_pDepthDistortionLT[i];
				*pDepth = id < bufferLength ? depthData[id] : 0;
			}
		}
		
		//�����ͼӳ�䵽��ɫͼ�ռ�
		if (SUCCEEDED(hr)) {
			hr = m_pMapper->MapDepthFrameToColorSpace(m_cDepthWidth * m_cDepthHeight, m_pDepthImagePixelBuffer, m_cDepthWidth * m_cDepthHeight, m_pColorCoordinates);
		}
		
		if (SUCCEEDED(hr)) {
			//�õ������ͼ����Ĳ�ɫͼ
			MapColorToDepth();

			//kinectfusion����
			//mProcessDepth();

			for (int i = 0; i < m_cDepthImagePixels; i++) {
						i_depthToRgb.data[i * 4] = m_pResampledColorImageDepthAligned->pFrameBuffer->pBits[i * 4];
						i_depthToRgb.data[i * 4 + 1] = m_pResampledColorImageDepthAligned->pFrameBuffer->pBits[i * 4 + 1];
						i_depthToRgb.data[i * 4 + 2] = m_pResampledColorImageDepthAligned->pFrameBuffer->pBits[i * 4 + 2];
						i_depthToRgb.data[i * 4 + 3] = m_pResampledColorImageDepthAligned->pFrameBuffer->pBits[i * 4 + 3];
			}
			//imwrite("rgbimg", i_rgb);
			imshow("rgb2depth", i_depthToRgb);
			waitKey(1000);
		}

		//if (SUCCEEDED(hr)) 		//������ʾ���ͼ
		//	processDepth();
		//UINT nBufferSize = m_cDepthImagePixels;
		//if (SUCCEEDED(hr))								//ÿ�����ͼ�϶�Ӧһ�����꣨��ɫͼ������꣩����ʾ�˵�Ͷ�Ӧ�Ĳ�ɫͼ�������ϵĵ��Ӧ
		//	hr = m_pMapper->MapDepthFrameToColorSpace(nBufferSize, depthData, nBufferSize, m_pColorCoordinates);
		//if (SUCCEEDED(hr))		//	���Խ����ͼת���ɶ�Ӧ�Ĳ�ɫͼ
		//	precessDepthToColor();
		//if (SUCCEEDED(hr))
		//	hr = m_pMapper->MapDepthFrameToCameraSpace(nBufferSize, depthData, nBufferSize, m_pCameraCoordinates);
		//if (SUCCEEDED(hr))			//������ʾ������Ϣ
		//	processDepthToCloud();
		//if (SUCCEEDED(hr))
		//{
		//	//copy and remap depth
		//	const UINT bufferLength = m_cDepthImagePixels;
		//	UINT16 * pDepth = m_pDepthImagePixelBuffer;
		//	for (UINT i = 0; i < bufferLength; i++, pDepth++)
		//	{
		//		const UINT id = m_pDepthDistortionLT[i];
		//		*pDepth = id < bufferLength ? depthData[id] : 0;
		//	}
		//	mProcessDepth();
		//}
	}
	SafeRelease(pDepthFrame);
	SafeRelease(pColorFrame);
	SafeRelease(m_pColorFrameReference);
	SafeRelease(m_pDepthFrameReference);
	SafeRelease(pMultiFrame);
}
/// <summary>
/// Adjust color to the same space as depth
/// </summary>
/// <returns>S_OK for success, or failure code</returns>
HRESULT KinectFusion::MapColorToDepth()
{
	HRESULT hr = S_OK;

	if (nullptr == m_pColorImage || nullptr == m_pResampledColorImageDepthAligned
		|| nullptr == m_pColorCoordinates || nullptr == m_pDepthVisibilityTestMap)
	{
		return E_FAIL;
	}

	NUI_FUSION_BUFFER *srcColorBuffer = m_pColorImage->pFrameBuffer;
	NUI_FUSION_BUFFER *destColorBuffer = m_pResampledColorImageDepthAligned->pFrameBuffer;

	if (nullptr == srcColorBuffer || nullptr == destColorBuffer)
	{
		cout << "Error accessing color textures." << endl;
		return E_NOINTERFACE;
	}

	if (FAILED(hr) || srcColorBuffer->Pitch == 0)
	{
		cout << "Error accessing color texture pixels." << endl;
		return  E_FAIL;
	}

	if (FAILED(hr) || destColorBuffer->Pitch == 0)
	{
		cout << "Error accessing color texture pixels." << endl;
		return  E_FAIL;
	}

	int *rawColorData = reinterpret_cast<int*>(srcColorBuffer->pBits);
	int *colorDataInDepthFrame = reinterpret_cast<int*>(destColorBuffer->pBits);

	// Get the coordinates to convert color to depth space
	hr = m_pMapper->MapDepthFrameToColorSpace(NUI_DEPTH_RAW_WIDTH * NUI_DEPTH_RAW_HEIGHT, m_pDepthImagePixelBuffer,
		NUI_DEPTH_RAW_WIDTH * NUI_DEPTH_RAW_HEIGHT, m_pColorCoordinates);

	if (FAILED(hr))
	{
		return hr;
	}

	// construct dense depth points visibility test map so we can test for depth points that are invisible in color space
	const UINT16* const pDepthEnd = m_pDepthImagePixelBuffer + NUI_DEPTH_RAW_WIDTH * NUI_DEPTH_RAW_HEIGHT;
	const ColorSpacePoint* pColorPoint = m_pColorCoordinates;
	const UINT testMapWidth = UINT(m_cColorWidth >> cVisibilityTestQuantShift);
	const UINT testMapHeight = UINT(m_cColorHeight >> cVisibilityTestQuantShift);
	ZeroMemory(m_pDepthVisibilityTestMap, testMapWidth * testMapHeight * sizeof(UINT16));
	for (const UINT16* pDepth = m_pDepthImagePixelBuffer; pDepth < pDepthEnd; pDepth++, pColorPoint++)
	{
		const UINT x = UINT(pColorPoint->X + 0.5f) >> cVisibilityTestQuantShift;
		const UINT y = UINT(pColorPoint->Y + 0.5f) >> cVisibilityTestQuantShift;
		if (x < testMapWidth && y < testMapHeight)
		{
			const UINT idx = y * testMapWidth + x;
			const UINT16 oldDepth = m_pDepthVisibilityTestMap[idx];
			const UINT16 newDepth = *pDepth;
			if (!oldDepth || oldDepth > newDepth)
			{
				m_pDepthVisibilityTestMap[idx] = newDepth;
			}
		}
	}


	// Loop over each row and column of the destination color image and copy from the source image
	// Note that we could also do this the other way, and convert the depth pixels into the color space, 
	// avoiding black areas in the converted color image and repeated color images in the background
	// However, then the depth would have radial and tangential distortion like the color camera image,
	// which is not ideal for Kinect Fusion reconstruction.

	if (m_bMirrorDepthFrame)
	{
		for (unsigned int y = 0; y < m_cDepthHeight; y++)
		{
			const UINT depthWidth = m_cDepthWidth;
			const UINT depthImagePixels = m_cDepthImagePixels;
			const UINT colorHeight = m_cColorHeight;
			const UINT colorWidth = m_cColorWidth;
			const UINT testMapWidth = UINT(colorWidth >> cVisibilityTestQuantShift);

			UINT destIndex = y * depthWidth;
			for (UINT x = 0; x < depthWidth; ++x, ++destIndex)
			{
				int pixelColor = 0;
				const UINT mappedIndex = m_pDepthDistortionLT[destIndex];
				if (mappedIndex < depthImagePixels)
				{
					// retrieve the depth to color mapping for the current depth pixel
					const ColorSpacePoint colorPoint = m_pColorCoordinates[mappedIndex];

					// make sure the depth pixel maps to a valid point in color space
					const UINT colorX = (UINT)(colorPoint.X + 0.5f);
					const UINT colorY = (UINT)(colorPoint.Y + 0.5f);
					if (colorX < colorWidth && colorY < colorHeight)
					{
						const UINT16 depthValue = m_pDepthImagePixelBuffer[mappedIndex];
						const UINT testX = colorX >> cVisibilityTestQuantShift;
						const UINT testY = colorY >> cVisibilityTestQuantShift;
						const UINT testIdx = testY * testMapWidth + testX;
						const UINT16 depthTestValue = m_pDepthVisibilityTestMap[testIdx];
						_ASSERT(depthValue >= depthTestValue);
						if (depthValue - depthTestValue < cDepthVisibilityTestThreshold)
						{
							// calculate index into color array
							const UINT colorIndex = colorX + (colorY * colorWidth);
							pixelColor = rawColorData[colorIndex];
						}
					}
				}
				colorDataInDepthFrame[destIndex] = pixelColor;
			}
		}
	}
	else
	{
		for (unsigned int y = 0; y < m_cDepthHeight; y++)
		{
			const UINT depthWidth = m_cDepthWidth;
			const UINT depthImagePixels = m_cDepthImagePixels;
			const UINT colorHeight = m_cColorHeight;
			const UINT colorWidth = m_cColorWidth;
			const UINT testMapWidth = UINT(colorWidth >> cVisibilityTestQuantShift);

			// Horizontal flip the color image as the standard depth image is flipped internally in Kinect Fusion
			// to give a viewpoint as though from behind the Kinect looking forward by default.
			UINT destIndex = y * depthWidth;
			UINT flipIndex = destIndex + depthWidth - 1;
			for (UINT x = 0; x < depthWidth; ++x, ++destIndex, --flipIndex)
			{
				int pixelColor = 0;
				const UINT mappedIndex = m_pDepthDistortionLT[destIndex];
				if (mappedIndex < depthImagePixels)
				{
					// retrieve the depth to color mapping for the current depth pixel
					const ColorSpacePoint colorPoint = m_pColorCoordinates[mappedIndex];

					// make sure the depth pixel maps to a valid point in color space
					const UINT colorX = (UINT)(colorPoint.X + 0.5f);
					const UINT colorY = (UINT)(colorPoint.Y + 0.5f);
					if (colorX < colorWidth && colorY < colorHeight)
					{
						const UINT16 depthValue = m_pDepthImagePixelBuffer[mappedIndex];
						const UINT testX = colorX >> cVisibilityTestQuantShift;
						const UINT testY = colorY >> cVisibilityTestQuantShift;
						const UINT testIdx = testY * testMapWidth + testX;
						const UINT16 depthTestValue = m_pDepthVisibilityTestMap[testIdx];
						_ASSERT(depthValue >= depthTestValue);
						if (depthValue - depthTestValue < cDepthVisibilityTestThreshold)
						{
							// calculate index into color array
							const UINT colorIndex = colorX + (colorY * colorWidth);
							pixelColor = rawColorData[colorIndex];
						}
					}
				}
				colorDataInDepthFrame[flipIndex] = pixelColor;
			}
		}
	}

	return hr;
}
void KinectFusion::precessDepthToColor() {
	
	for (int i = 0; i < m_cDepthImagePixels; i++) {
		ColorSpacePoint p = m_pColorCoordinates[i];
		if (p.X != -std::numeric_limits<float>::infinity() && p.Y != -std::numeric_limits<float>::infinity()) {
			int colorX = static_cast<int>(p.X + 0.5f);
			int colorY = static_cast<int>(p.Y + 0.5f);
			if ((colorX >= 0 && colorX < 1920) && (colorY >= 0 && colorY < 1080))
			{
				i_depthToRgb.data[i * 4] = i_rgb.data[(colorY * 1920 + colorX) * 4];
				i_depthToRgb.data[i * 4 + 1] = i_rgb.data[(colorY * 1920 + colorX) * 4 + 1];
				i_depthToRgb.data[i * 4 + 2] = i_rgb.data[(colorY * 1920 + colorX) * 4 + 2];
				i_depthToRgb.data[i * 4 + 3] = i_rgb.data[(colorY * 1920 + colorX) * 4 + 3];
			}
		}
	}
	//imwrite("rgbimg", i_rgb);
	imshow("rgb2depth", i_depthToRgb);
	waitKey(5000);
}
void KinectFusion::processDepth() {
	for (int i = 0; i < m_cDepthImagePixels; i++)
	{
		// 0-255���ͼ��Ϊ����ʾ���ԣ�ֻȡ������ݵĵ�8λ
		BYTE intensity = static_cast<BYTE>(depthData[i] % 256);
		reinterpret_cast<BYTE*>(i_depth.data)[i] = intensity;
	}
	imshow("depth", i_depth);
	waitKey(100);
}
void KinectFusion::processDepthToCloud() {
	int count = 0;
	for (int i = 0; i < m_cDepthImagePixels; i++)
	{
		CameraSpacePoint p = m_pCameraCoordinates[i];
		if (p.X != -std::numeric_limits<float>::infinity() && p.Y != -std::numeric_limits<float>::infinity() && p.Z != -std::numeric_limits<float>::infinity())
		{
			count++;
		}
	}
	ofstream ofs(".\\out.ply");
	string num;
	stringstream ss;
	ss << count;
	ss >> num;
	string str = "ply\nformat ascii 1.0\nelement face 0\n property list uchar int vertex_indices\nelement vertex " + string(num) + "\nproperty float x\nproperty float y\nproperty float z\nproperty float nx\nproperty float ny\nproperty float nz\nproperty uchar diffuse_red\nproperty uchar diffuse_green\nproperty uchar diffuse_blue\nproperty uchar alpha\nend_header\n";
	ofs << str;
	for (int i = 0; i < m_cDepthImagePixels; i++)
	{
		CameraSpacePoint p = m_pCameraCoordinates[i];
		if (p.X != -std::numeric_limits<float>::infinity() && p.Y != -std::numeric_limits<float>::infinity() && p.Z != -std::numeric_limits<float>::infinity())
		{
			float cameraX = static_cast<float>(p.X);
			float cameraY = static_cast<float>(p.Y);
			float cameraZ = static_cast<float>(p.Z);
			ofs << cameraX << ' ' << cameraY << ' ' << cameraZ << ' ' << static_cast<int>(0) << ' ' << static_cast<int>(0) << ' ' << static_cast<int>(0) << ' ' << static_cast<int>(i_depthToRgb.data[i * 4 + 0]) << ' ' << static_cast<int>(i_depthToRgb.data[i * 4 + 1]) << ' ' << static_cast<int>(i_depthToRgb.data[i * 4 + 2]) << ' ' << static_cast<int>(255) << endl;
		}
	}
	ofs.close();
	cout << "ok" << endl;
}
void KinectFusion::mProcessDepth() {
	if (m_bInitializeError)
		return;
	HRESULT hr = S_OK;
	if (nullptr == m_pVolume)
	{
		cout << "Kinect Fusion reconstruction volume not initialized. Please try reducing volume size or restarting." << endl;
		return;
	}
	//��ԭ������ݹ��측�����ͼ����
	hr = m_pVolume->DepthToDepthFloatFrame(m_pDepthImagePixelBuffer, m_cDepthImagePixels * sizeof(UINT16), m_pDepthFloatImage, m_fMinDepthThreshold, m_fMaxDepthThreshold, m_bMirrorDepthFrame);
	//hr = m_pVolume->DepthToDepthFloatFrame(depthData, m_cDepthImagePixels * sizeof(UINT16), m_pDepthFloatImage, m_fMinDepthThreshold, m_fMaxDepthThreshold, true);
	if (FAILED(hr))
	{
		cout << "Kinect Fusion NuiFusionDepthToDepthFloatFrame call failed." << endl;
		return;
	}

	 //ƽ���������
	hr = m_pVolume->SmoothDepthFloatFrame(m_pDepthFloatImage, m_pSmoothDepthFloatImage, 1, 0.03f);
	if (FAILED(hr)){
		cout << "Kinect Fusion SmoothDepthFloatFrame call failed." << endl;
		return;
	}

	//����ǰ֡�� ���� camera tracking �� update the Kinect Fusion Volume
	FLOAT alignmentEnergy = 1.0f;
	if (SUCCEEDED(hr)) 
		hr = m_pVolume->ProcessFrame(m_pSmoothDepthFloatImage, m_pResampledColorImageDepthAligned, NUI_FUSION_DEFAULT_ALIGN_ITERATION_COUNT, m_cMaxIntegrationWeight, NUI_FUSION_DEFAULT_COLOR_INTEGRATION_OF_ALL_ANGLES, &alignmentEnergy, &m_worldToCameraTransform);
	
	// ��� camera tracking �Ƿ�ʧ��. 
	if (FAILED(hr))
	{
		if (hr == E_NUI_FUSION_TRACKING_ERROR)
		{
			m_cLostFrameCounter++;
			m_bTrackingFailed = true;
			cout << "Kinect Fusion camera tracking failed! Align the camera to the last tracked position. " << endl;
		}
		else
		{
			cout << "Kinect Fusion ProcessFrame call failed!" << endl;	
			return;
		}		
	}
	else
	{
		Matrix4 calculatedCameraPose;
		hr = m_pVolume->GetCurrentWorldToCameraTransform(&calculatedCameraPose);
		if (SUCCEEDED(hr))
		{
			if (m_bTrackingFailed)
				cout << "lostFrameCounter:" << m_cLostFrameCounter << endl;
			// Set the pose
			m_worldToCameraTransform = calculatedCameraPose;
			m_cLostFrameCounter = 0;
			m_bTrackingFailed = false;

		}
	}

	// �������ͶӰ����֡
	// Raycast all the time, even if we camera tracking failed, to enable us to visualize what is happening with the system
	hr = m_pVolume->CalculatePointCloud(m_pPointCloud, m_pCapturedSurfaceColor, &m_worldToCameraTransform);
	if (FAILED(hr))
	{
		cout << "Kinect Fusion CalculatePointCloud call failed." << endl;
		return;
	}

	// ������Ⱦ
	// Map X axis to blue channel, Y axis to green channel and Z axis to red channel,
	// normalizing each to the range [0, 1].
	Matrix4 worldToBGRTransform = { 0.0f };
	worldToBGRTransform.M11 = m_reconstructionParams.voxelsPerMeter / m_reconstructionParams.voxelCountX;
	worldToBGRTransform.M22 = m_reconstructionParams.voxelsPerMeter / m_reconstructionParams.voxelCountY;
	worldToBGRTransform.M33 = m_reconstructionParams.voxelsPerMeter / m_reconstructionParams.voxelCountZ;
	worldToBGRTransform.M41 = 0.5f;
	worldToBGRTransform.M42 = 0.5f;
	worldToBGRTransform.M43 = 0.0f;
	worldToBGRTransform.M44 = 1.0f;
	hr = NuiFusionShadePointCloud(m_pPointCloud, &m_worldToCameraTransform, &worldToBGRTransform, m_pShadedSurface, nullptr);
	if (FAILED(hr))
	{
		cout << "Kinect Fusion NuiFusionShadePointCloud call failed." << endl;
		return;
	}
	m_cFrameCounter++;
}
void KinectFusion::getPointsCloud() {
	//mesh
	INuiFusionColorMesh* colorMesh;
	HRESULT hr = m_pVolume->CalculateMesh(1, &colorMesh);
	if (FAILED(hr)) {
		cout << "CalculateMesh failed!" << endl;
		return;
	}
	unsigned int count = colorMesh->VertexCount();
	unsigned int nCount = colorMesh->NormalCount();
	unsigned int tCount = colorMesh->TriangleVertexIndexCount()/3;
	cout << "vertex :" << count << endl;
	cout << "normal:" << nCount << endl;
	cout << "triangles:" << tCount << endl;
	const Vector3 *pVertices;// = new Vector3[count];
	hr = colorMesh->GetVertices(&pVertices);
	if (FAILED(hr)) {
		cout << "get Vertices failed!" << endl;
		return;
	}
	const Vector3* pNormals;
	hr = colorMesh->GetNormals(&pNormals);
	if (FAILED(hr)) {
		cout << "get Normals failed!" << endl;
		return;
	}
	const int* colors;
	hr = colorMesh->GetColors(&colors);
	if (FAILED(hr)) {
		cout << "get Colors failed!" << endl;
		return;
	}
	
	ofstream ofs(".\\result.ply");
	string num;
	stringstream ss;
	ss << count;
	ss >> num;
	string str = "ply\nformat ascii 1.0\nelement vertex " + string(num) + "\nproperty float x\nproperty float y\nproperty float z\nproperty float nx\nproperty float ny\nproperty float nz\nproperty uchar diffuse_red\nproperty uchar diffuse_green\nproperty uchar diffuse_blue\nproperty uchar alpha\n";
	stringstream ss2;
	string num2;
	ss2 << tCount;
	ss2 >> num2;
	str += "element face " + string(num2) + "\nproperty list uchar int vertex_index\nend_header\n";
	ofs << str;
	for (unsigned i = 0; i < count; i++)
	{
		ofs << pVertices[i].x << ' ' << pVertices[i].y << ' ' << pVertices[i].z << ' ' << pNormals[i].x << ' ' << pNormals[i].y << ' ' << pNormals[i].z << ' ' << ((colors[i] >> 16) & 255) << ' ' << ((colors[i] >> 8) & 255) << ' ' << ((colors[i] >> 16) & 255) << ' ' << static_cast<int>(255) << endl;
	}
	for (unsigned i = 0; i < nCount; i= i + 3) {
		ofs << static_cast<int>(3) << ' ' << static_cast<int>(i) << ' ' << static_cast<int>(i + 1) << ' ' << static_cast<int>(i + 2) << endl;
	}
	ofs.close();
}
void KinectFusion::processResult() {
	// Draw the shaded raycast volume image
	//Mat showImg(m_cDepthHeight, m_cDepthWidth, CV_8UC4);
	//BYTE * pBuffer = m_pShadedSurface->pFrameBuffer->pBits;
	//unsigned long cbImage  = m_cDepthWidth * m_cDepthHeight * cBytesPerPixel;
	//for (int i = 0; i < m_cDepthImagePixels; i++) {
	//	showImg.data[i * 4] = pBuffer[i * 4];
	//	showImg.data[i * 4 + 1] = static_cast<uchar>(pBuffer[i * 4 + 1]);
	//	showImg.data[i * 4 + 2] = static_cast<uchar>(pBuffer[i * 4 + 2]);
	//	showImg.data[i * 4 + 3] = static_cast<uchar>(pBuffer[i * 4 + 3]);
	//}
	//imshow("kinect fusion", showImg);
	//waitKey(5000);

	//BYTE * pBuffer = m_pShadedSurface->pFrameBuffer->pBits;
	//int count = 0;
	//for (int i = 0; i < m_cDepthImagePixels; i++)
	//{
	//	float x = static_cast<float>(pBuffer[i * 4]);
	//	float y = static_cast<float>(pBuffer[i * 4 + 1]);
	//	float z = static_cast<float>(pBuffer[i * 4 + 2]);	
	//	float w = static_cast<float>(pBuffer[i * 4 + 3]);
	//	if (x != 0 || y != 0 || z != 0)
	//	{
	//		count++;
	//	}
	//}
	//ofstream ofs(".\\result.ply");
	//string num;
	//stringstream ss;
	//ss << count;
	//ss >> num;
	//string str = "ply\nformat ascii 1.0\nelement face 0\n property list uchar int vertex_indices\nelement vertex " + string(num) + "\nproperty float x\nproperty float y\nproperty float z\nproperty float nx\nproperty float ny\nproperty float nz\nproperty uchar diffuse_red\nproperty uchar diffuse_green\nproperty uchar diffuse_blue\nproperty uchar alpha\nend_header\n";
	//ofs << str;
	//for (int i = 0; i < m_cDepthImagePixels; i++)
	//{
	//	float x = static_cast<float>(pBuffer[i * 4]);
	//	float y = static_cast<float>(pBuffer[i * 4 + 1]);
	//	float z = static_cast<float>(pBuffer[i * 4 + 2]);
	//	float w = static_cast<float>(pBuffer[i * 4 + 3]);
	//	if (x != 0 || y != 0 || z != 0)
	//	{
	//		//ofs << x << ' ' << y << ' ' << z << ' ' << static_cast<int>(0) << ' ' << static_cast<int>(0) << ' ' << static_cast<int>(0) << ' ' << static_cast<int>(i_depthToRgb.data[i * 4 + 0]) << ' ' << static_cast<int>(i_depthToRgb.data[i * 4 + 1]) << ' ' << static_cast<int>(i_depthToRgb.data[i * 4 + 2]) << ' ' << static_cast<int>(255) << endl;
	//		ofs << x << ' ' << y << ' ' << z << ' ' << static_cast<int>(0) << ' ' << static_cast<int>(0) << ' ' << static_cast<int>(0) << ' ' << static_cast<int>(255) << ' ' << static_cast<int>(255) << ' ' << static_cast<int>(255) << ' ' << static_cast<int>(255) << endl;
	//	}
	//}
	//ofs.close();

	cout << "ok" << endl;

}
int  main() {
	
	KinectFusion kf;
	HRESULT hr = kf.CreateFirstConnected();	
	if (FAILED(hr)) {
		cout << "111" << endl;
		return 0;
	}
	hr = kf.InitializeKinectFusion();
	if (FAILED(hr)) {
		cout << "1112" << endl;
		return 0;
	}
	//kf.update();
	//getchar();
	//kf.update();	
	//for (int i = 0; i < 50; i++) {
	//	kf.update();
	//	waitKey(10);
	//}
	unsigned int cc = 0;
	while (1) {
		cc++;
		kf.update();
		if ((kf.m_bTrackingFailed && kf.m_cLostFrameCounter >= 50 ) || kf.m_cFrameCounter >= 300)
			break;
	}
	cout << "1114" << endl;
	//kf.processResult();
	kf.getPointsCloud();
	cout << "cc number" << cc << endl;
	cout << "֡����" << kf.m_cFrameCounter << endl;
	//while (1) {
	//	kf.update();
	//	waitKey(2000);
	//}
	return 0;
}
