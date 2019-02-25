package coordtarns

import (
	"gonum.org/v1/gonum/mat"
	"math"
)

const Re = 6378.137             //地球赤道半径2002.4.7
const Ra = 6378137.0            //WGS84椭球长半轴
const Rb = 6356752.314245179497 //WGS84椭球短半轴
const e1 = 0.081819190842621    //第一偏心率，计算公式为e1=sqrt(Ra*Ra-Rb*Rb)/Ra
const e22 = 0.082094437949656   //第二偏心率
const e2 = 0.993305458
const M_PI = math.Pi

type radar struct {
	radar_jd float64
	radar_wd float64
	radar_h  float64
}

func (this *radar) rae_xyz(r float64, a float64, e float64) (xx, yy, zz float64) {
	var rr float64
	if r < 0.0000000001 {
		rr = 0.00000001
	} else {
		rr = r
	}

	xx = rr * math.Cos(e*math.Pi/180.) * math.Sin(a*math.Pi/180.) / 1000.
	yy = rr * math.Cos(e*math.Pi/180.) * math.Cos(a*math.Pi/180.) / 1000.
	zz = rr * math.Sin(e*math.Pi/180.) / 1000.0
	return
}

func (this *radar) xyz_rae(xx, yy, zz float64) (r, a, e float64) {

	r = math.Sqrt(xx*xx + yy*yy + zz*zz)
	a = math.Atan2(xx, yy) * 180. / math.Pi
	if a < 0 {
		a += 360.0
	}
	e = math.Asin(zz/r) * 180. / math.Pi
	r = r * 1000.0
	return
}

//目标在雷达极坐标系转换到WGS84空间直角坐标系
func (this *radar) xyz_XYZ(x, y, z float64) (XX, YY, ZZ float64) {

	R2 := mat.NewDense(3, 3, nil)
	T2 := mat.NewDense(3, 1, nil)
	mTargetWGS84 := mat.NewDense(3, 1, nil)
	mTargetRadar := mat.NewDense(3, 1, nil)

	var rj, rw, rh float64
	var pj, qj, pw, qw, N float64

	rj = this.radar_jd * M_PI / 180.0
	rw = this.radar_wd * M_PI / 180.0
	rh = this.radar_h / 1000.0

	pj = math.Sin(rj)
	qj = math.Cos(rj)
	pw = math.Sin(rw)
	qw = math.Cos(rw)
	N = Ra / (1000.0 * math.Sqrt(1-e1*e1*pw*pw))

	mTargetRadar.Set(0, 0, x)
	mTargetRadar.Set(1, 0, y)
	mTargetRadar.Set(2, 0, z)

	R2.Set(0, 0, -pj)
	R2.Set(0, 1, qj)
	R2.Set(0, 2, 0)

	R2.Set(1, 0, -pw*qj)
	R2.Set(1, 1, -pw*pj)
	R2.Set(1, 2, qw)

	R2.Set(2, 0, qw*qj)
	R2.Set(2, 1, qw*pj)
	R2.Set(2, 2, pw)

	T2.Set(0, 0, (N+rh)*qw*qj)
	T2.Set(1, 0, (N+rh)*qw*pj)
	T2.Set(2, 0, (N*(1-e1*e1)+rh)*pw)

	tmp1 := mat.NewDense(3, 1, nil)
	tmp1.Product(R2.T(), mTargetRadar)
	mTargetWGS84.Add(tmp1, T2)

	XX = mTargetWGS84.At(0, 0)
	YY = mTargetWGS84.At(1, 0)
	ZZ = mTargetWGS84.At(2, 0)
	return

}

func (this *radar) XYZ_jwh(XX, YY, ZZ float64) (jd, wd, h float64) {
	var N, U float64
	XX = XX * 1000.0
	YY = YY * 1000.0
	ZZ = ZZ * 1000.0
	p := math.Sqrt(XX*XX + YY*YY)

	jd = math.Atan2(YY, XX) * 180.0 / M_PI

	U = math.Atan2(ZZ, p*math.Sqrt((1-e1*e1)))
	wd = math.Atan2(ZZ+Rb*e22*e22*math.Sin(U)*math.Sin(U)*math.Sin(U), p-e1*e1*Ra*math.Cos(U)*math.Cos(U)*math.Cos(U))
	N = Ra / math.Sqrt(1-e1*e1*math.Sin(wd)*math.Sin(wd))
	h = p/math.Cos(wd) - N

	wd = wd * 180.0 / M_PI
	h = h / 1000.0
	return
}

func (this *radar) XYZ_xyz(XX, YY, ZZ float64) (xx, yy, zz float64) {

	var rj, rw, rh float64
	var pj, qj, pw, qw float64

	mm := mat.NewDense(3, 3, nil)
	mX := mat.NewDense(3, 1, nil)
	mx := mat.NewDense(3, 1, nil)
	mc := mat.NewDense(3, 1, nil)

	rj = this.radar_jd * M_PI / 180.0
	rw = this.radar_wd * M_PI / 180.0
	rh = this.radar_h

	pj = math.Sin(rj)
	qj = math.Cos(rj)
	pw = math.Sin(rw)
	qw = math.Cos(rw)

	//雷达站在WGS直角坐标下坐标
	N0 := Ra / math.Sqrt(1-e1*e1*pw*pw)
	X0 := (N0 + rh) * qw * qj / 1000.0
	Y0 := (N0 + rh) * qw * pj / 1000.0
	Z0 := (N0*(1-e1*e1) + rh) * pw / 1000.0

	mm.Set(0, 0, -pj)
	mm.Set(0, 1, -pw*qj)
	mm.Set(0, 2, qw*qj)

	mm.Set(1, 0, qj)
	mm.Set(1, 1, -pw*pj)
	mm.Set(1, 2, qw*pj)

	mm.Set(2, 0, 0)
	mm.Set(2, 1, qw)
	mm.Set(2, 2, pw)

	mc.Set(0, 0, X0)
	mc.Set(1, 0, Y0)
	mc.Set(2, 0, Z0)

	mX.Set(0, 0, XX)
	mX.Set(1, 0, YY)
	mX.Set(2, 0, ZZ)

	D := mat.NewDense(3, 1, nil)
	D.Sub(mX, mc)

	mx.Product(mm.T(), D)

	xx = mx.At(0, 0)
	yy = mx.At(1, 0)
	zz = mx.At(2, 0)
	return

}

func (this *radar) jwh_XYZ(jd, wd, h float64) (XX, YY, ZZ float64) {
	//目标在WGS直角坐标下坐标（东北天）
	L := jd * M_PI / 180.0
	B := wd * M_PI / 180.0
	H := h * 1000.0
	N := Ra / math.Sqrt(1-e1*e1*math.Sin(B)*math.Sin(B))
	XX = (N + H) * math.Cos(B) * math.Cos(L) / 1000.0
	YY = (N + H) * math.Cos(B) * math.Sin(L) / 1000.0
	ZZ = (N*(1-e1*e1) + H) * math.Sin(B) / 1000.0
	return
}

func (this *radar) jwh_rae(jd, wd, h float64) (r, a, e float64) {
	XX, YY, ZZ := this.jwh_XYZ(jd, wd, h/1000.0) //度，公里--->公里
	x, y, z := this.XYZ_xyz(XX, YY, ZZ)          //公里--->公里
	r, a, e = this.xyz_rae(x, y, z)              //公里--->米、度
	return
}

func (this *radar) rae_jwh(r, a, e float64) (jd, wd, h float64) {
	x, y, z := this.rae_xyz(r, a, e)
	XX, YY, ZZ := this.xyz_XYZ(x, y, z)
	jd, wd, h = this.XYZ_jwh(XX, YY, ZZ)
	h = h * 1000.0
	return
}
