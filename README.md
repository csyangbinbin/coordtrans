# coordtrans
计算地球上两点间的距离、方位、俯仰

#示例代码
```go
package main
import (
	"fmt"
	 "github.com/csyangbinbin/coordtrans"
)

func main() {
	LocJd := 121.15
	locaWd := 37.5
	localH := 40.0
  
	targetJw := 121.6
	targetWd := 37.6
	targetH := 60.0

	P := coordtarns.JWH2RAE_S(coordtarns.GeoCoord{LocJd, locaWd, localH},
		coordtarns.GeoCoord{targetJw, targetWd, targetH})

	G := coordtarns.RAE2JWH_S(coordtarns.GeoCoord{LocJd, locaWd, localH}, P)

	fmt.Print(P, G)

}
```
