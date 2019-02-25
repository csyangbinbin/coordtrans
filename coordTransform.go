package coordtarns


type GeoCoord struct {
	Lon float64 //经度
	Lat float64 //纬度
	Alt float64 //高度(单位:米)
}

type PolarCoordstruct struct {
	R float64 //距离
	A float64 //方位
	E float64 //俯仰
}

func JWH2RAE(LocJd, LocWd, LocH, targetJw, TargeWd, TargetH float64) (r, a, e float64) {
	gps_radar := &radar{LocJd, LocWd, LocH}
	return gps_radar.jwh_rae(targetJw, TargeWd, TargetH)
}

func RAE2JWH(LocJd, LocWd, LocH, r, a, e float64) (targetJw, TargeWd, TargetH float64) {
	gps_radar := &radar{LocJd, LocWd, LocH}
	return gps_radar.rae_jwh(r, a, e)
}


func JWH2RAE_S(ref GeoCoord, target GeoCoord) (PolarCoordstruct) {
	gps_radar := &radar{ref.Lon, ref.Lat, ref.Alt}
	r,a,e :=  gps_radar.jwh_rae(target.Lon ,target.Lat ,target.Alt)
	return PolarCoordstruct{r,a,e}
}

func RAE2JWH_S(ref GeoCoord, target PolarCoordstruct) (GeoCoord) {
	gps_radar := &radar{ref.Lon, ref.Lat, ref.Alt}
	j,w,h := gps_radar.rae_jwh(target.R ,target.A,target.E)
	return GeoCoord{j,w,h}
}