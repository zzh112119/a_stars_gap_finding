import numpy as np
from sklearn.cluster import KMeans
from sklearn.cluster import DBSCAN
import time
from sklearn.mixture import GaussianMixture
from sklearn.decomposition import PCA
import matplotlib.pyplot as plt
import math

#ONE FRAME OF RESULT
ranges = [3.941844940185547, 3.896338701248169, 3.845069646835327, 3.786914110183716, 3.7543740272521973, 3.7206435203552246, 3.671466827392578, 3.6442089080810547, 3.6109843254089355, 3.569035053253174, 3.5329439640045166, 3.498764991760254, 3.472842216491699, 3.4613590240478516, 3.407747983932495, 3.3656444549560547, 3.351210832595825, 3.3178842067718506, 3.291577100753784, 3.2606327533721924, 3.1984386444091797, 3.1985881328582764, 3.1716530323028564, 3.1357548236846924, 3.0828559398651123, 3.083807945251465, 3.035472869873047, 3.024015426635742, 2.9723336696624756, 2.951852798461914, 2.926363706588745, 2.8963396549224854, 2.883288621902466, 2.844021797180176, 2.8189470767974854, 2.806258201599121, 2.7814016342163086, 2.767090320587158, 2.7436580657958984, 2.7134640216827393, 2.6861865520477295, 2.6547560691833496, 2.647282361984253, 2.614928960800171, 2.597165584564209, 2.570178985595703, 2.5806853771209717, 2.5460851192474365, 2.5389440059661865, 2.505483388900757, 2.4856374263763428, 2.4692025184631348, 2.4566826820373535, 2.4204299449920654, 2.391832113265991, 2.393192768096924, 2.353966474533081, 2.3235771656036377, 2.3309357166290283, 2.301194906234741, 2.291944742202759, 2.25651216506958, 2.2553787231445312, 2.2516043186187744, 2.2065794467926025, 2.193878650665283, 2.202969551086426, 2.1830334663391113, 2.156740427017212, 2.1427173614501953, 2.1267263889312744, 2.124917507171631, 2.087801694869995, 2.079367160797119, 2.06406307220459, 2.044475555419922, 2.037252187728882, 2.038153886795044, 2.029130220413208, 1.997124433517456, 1.971001386642456, 1.9626740217208862, 1.960024356842041, 1.961480736732483, 1.9346641302108765, 1.922871470451355, 1.9018924236297607, 1.8881404399871826, 1.8988591432571411, 1.8863861560821533, 1.8884339332580566, 1.866662859916687, 1.8512287139892578, 1.8526054620742798, 1.8471922874450684, 1.8317222595214844, 1.8146758079528809, 1.7935971021652222, 1.8092412948608398, 1.7890863418579102, 1.7752020359039307, 1.7691001892089844, 1.7572815418243408, 1.7691599130630493, 1.7461800575256348, 1.731394648551941, 1.724342703819275, 1.7054331302642822, 1.6844956874847412, 1.6918842792510986, 1.6823363304138184, 1.6627223491668701, 1.6685993671417236, 1.6493555307388306, 1.6367158889770508, 1.6572914123535156, 1.6309237480163574, 1.6364216804504395, 1.6196430921554565, 1.600446343421936, 1.5901004076004028, 1.60916268825531, 1.5775126218795776, 1.5752708911895752, 1.5623747110366821, 1.5557563304901123, 1.5570412874221802, 1.565022587776184, 1.5384135246276855, 1.5136358737945557, 1.5066888332366943, 1.5089834928512573, 1.505247712135315, 1.5130841732025146, 1.4992263317108154, 1.4929413795471191, 1.4759055376052856, 1.4719589948654175, 1.4726097583770752, 1.470428705215454, 1.4634608030319214, 1.4455205202102661, 1.4598907232284546, 1.4396382570266724, 1.4403541088104248, 1.4390480518341064, 1.416985273361206, 1.4259405136108398, 1.420514702796936, 1.425799012184143, 1.3963871002197266, 1.3878107070922852, 1.400378704071045, 1.3916699886322021, 1.379465937614441, 1.384594202041626, 1.3546966314315796, 1.3770140409469604, 1.364549994468689, 1.368128776550293, 1.3529785871505737, 1.3466753959655762, 1.3528631925582886, 1.3585205078125, 1.3464099168777466, 1.3258291482925415, 1.3137774467468262, 1.3287285566329956, 1.3372349739074707, 1.3303159475326538, 1.3028805255889893, 1.3007471561431885, 1.306401252746582, 1.3142441511154175, 1.2886611223220825, 1.2863706350326538, 1.2932097911834717, 1.2813493013381958, 1.268646001815796, 1.272109866142273, 1.2773041725158691, 1.2735739946365356, 1.257426381111145, 1.27208411693573, 1.2532509565353394, 1.2477527856826782, 1.2494630813598633, 1.2553836107254028, 1.2420852184295654, 1.256798267364502, 1.2348929643630981, 1.2535581588745117, 1.241894006729126, 1.2167301177978516, 1.2266700267791748, 1.2183679342269897, 1.2298794984817505, 1.2302128076553345, 1.2165517807006836, 1.242861270904541, 1.2052456140518188, 1.1993855237960815, 1.1983569860458374, 1.215010166168213, 1.2026185989379883, 1.1758065223693848, 1.195019006729126, 1.186793565750122, 1.186207890510559, 1.1838608980178833, 1.1859456300735474, 1.1824709177017212, 1.1777124404907227, 1.1867773532867432, 1.179890751838684, 1.1594035625457764, 1.1617393493652344, 1.1704673767089844, 1.1665658950805664, 1.1653335094451904, 1.1614638566970825, 1.1658467054367065, 1.1432254314422607, 1.166420817375183, 1.1215916872024536, 1.1500240564346313, 1.1523959636688232, 1.144456386566162, 1.1477282047271729, 1.1429499387741089, 1.1485214233398438, 1.1208374500274658, 1.1391898393630981, 1.1284499168395996, 1.1380478143692017, 1.1287636756896973, 1.135244369506836, 1.120215892791748, 1.1306324005126953, 1.1202741861343384, 1.1115148067474365, 1.1299508810043335, 1.112094759941101, 1.1285463571548462, 1.1117697954177856, 1.1149173974990845, 1.112073302268982, 1.091370940208435, 1.1060045957565308, 1.108445405960083, 1.1278011798858643, 1.093361735343933, 1.0987942218780518, 1.0994163751602173, 1.0986361503601074, 1.0937789678573608, 1.1108710765838623, 1.1045148372650146, 1.094902515411377, 1.118543267250061, 1.068992018699646, 1.1038312911987305, 1.0723499059677124, 1.0862436294555664, 1.093201994895935, 1.0730295181274414, 1.073837399482727, 1.0833309888839722, 1.0727077722549438, 1.0768533945083618, 1.060013771057129, 1.0868598222732544, 1.085339069366455, 1.0835241079330444, 1.076189637184143, 1.0672590732574463, 1.0654124021530151, 1.0824781656265259, 1.068928599357605, 1.0605560541152954, 1.061720609664917, 1.0724287033081055, 1.0615211725234985, 1.0834811925888062, 1.0605970621109009, 1.066865086555481, 1.0709952116012573, 1.0616523027420044, 1.052483081817627, 1.0547922849655151, 1.0739880800247192, 1.064448356628418, 1.060906171798706, 1.0610631704330444, 1.0618598461151123, 1.0788631439208984, 1.0776392221450806, 1.0713938474655151, 1.047583818435669, 1.0627100467681885, 1.0406235456466675, 1.0688365697860718, 1.0628849267959595, 1.0546475648880005, 1.0516443252563477, 1.0543266534805298, 1.0803465843200684, 1.029636263847351, 1.041024088859558, 1.024074912071228, 1.0619616508483887, 1.0727123022079468, 1.0506420135498047, 1.0648616552352905, 1.0503588914871216, 1.0615394115447998, 1.0506354570388794, 1.0576876401901245, 1.0493943691253662, 1.0454773902893066, 1.04702889919281, 1.0272644758224487, 1.0500541925430298, 1.075330138206482, 1.0595189332962036, 1.0440129041671753, 1.0731897354125977, 1.0452239513397217, 1.0513614416122437, 1.0860508680343628, 1.0682576894760132, 1.067426323890686, 1.0681302547454834, 1.061862826347351, 1.0642331838607788, 1.0510870218276978, 1.056971788406372, 1.067688226699829, 1.0507080554962158, 1.0464990139007568, 1.0853668451309204, 1.0623259544372559, 1.0698614120483398, 1.0544761419296265, 1.0734031200408936, 1.051352858543396, 1.0570552349090576, 1.0832228660583496, 1.0703222751617432, 1.0651675462722778, 1.0608489513397217, 1.0590972900390625, 1.0569669008255005, 1.0514520406723022, 1.0643398761749268, 1.0765000581741333, 1.0751194953918457, 1.0686373710632324, 1.0755043029785156, 1.0635590553283691, 1.0598877668380737, 1.0634961128234863, 1.074302077293396, 1.0803660154342651, 1.0690324306488037, 1.0783838033676147, 1.0822474956512451, 1.0840023756027222, 1.087046504020691, 1.081225037574768, 1.0850642919540405, 1.0940901041030884, 1.0713918209075928, 1.0970797538757324, 1.0924172401428223, 1.0924649238586426, 1.0851378440856934, 1.0984768867492676, 1.1002330780029297, 1.1079916954040527, 1.0900598764419556, 1.1115505695343018, 1.093436598777771, 1.0895986557006836, 1.1065305471420288, 1.0962800979614258, 1.1052337884902954, 1.1137781143188477, 1.1040784120559692, 1.1175906658172607, 1.1160149574279785, 1.1107745170593262, 1.1045448780059814, 1.1143980026245117, 1.1140121221542358, 1.1102913618087769, 1.1217950582504272, 1.1178505420684814, 1.1207126379013062, 1.1266536712646484, 1.137831687927246, 1.110302209854126, 1.1226606369018555, 1.1296137571334839, 1.1337709426879883, 1.1429163217544556, 1.152915596961975, 1.1413215398788452, 1.1305334568023682, 1.1326608657836914, 1.1543245315551758, 1.1469097137451172, 1.1419588327407837, 1.1312774419784546, 1.1387333869934082, 1.183982014656067, 1.1748367547988892, 1.168508768081665, 1.1850100755691528, 1.1696765422821045, 1.1794089078903198, 1.1656444072723389, 1.189131498336792, 1.1681030988693237, 1.1895256042480469, 1.1770498752593994, 1.194697380065918, 1.1546378135681152, 1.1820068359375, 1.1896060705184937, 1.1891496181488037, 1.1924077272415161, 1.1816765069961548, 1.2016797065734863, 1.2041538953781128, 1.2027746438980103, 1.213653802871704, 1.1985561847686768, 1.2213239669799805, 1.221391201019287, 1.1999351978302002, 1.2054500579833984, 1.2402960062026978, 1.2252434492111206, 1.2373462915420532, 1.2347058057785034, 1.2344943284988403, 1.2424403429031372, 1.2584105730056763, 1.2465159893035889, 1.259589433670044, 1.241418480873108, 1.2464007139205933, 1.2606723308563232, 1.269803762435913, 1.272101879119873, 1.2760794162750244, 1.2762900590896606, 1.291680932044983, 1.2735847234725952, 1.2920666933059692, 1.2852239608764648, 1.277246356010437, 1.306024193763733, 1.3058676719665527, 1.2949538230895996, 1.3304307460784912, 1.3286398649215698, 1.311026692390442, 1.333191990852356, 1.3144766092300415, 1.35553777217865, 1.3256714344024658, 1.3422279357910156, 1.333707571029663, 1.357340693473816, 1.3729016780853271, 1.3813060522079468, 1.3540937900543213, 1.3798515796661377, 1.3594883680343628, 1.3922832012176514, 1.3845250606536865, 1.412016749382019, 1.4071203470230103, 1.4081302881240845, 1.3864147663116455, 1.400614619255066, 1.4255280494689941, 1.4158776998519897, 1.4349929094314575, 1.4265251159667969, 1.4561973810195923, 1.4415130615234375, 1.456910490989685, 1.465447187423706, 1.4566664695739746, 1.460355520248413, 1.476804494857788, 1.500783085823059, 1.4959717988967896, 1.4905554056167603, 1.5072540044784546, 1.5236154794692993, 1.5278100967407227, 1.5209343433380127, 1.5344158411026, 1.5525223016738892, 1.5592514276504517, 1.549723505973816, 1.5734678506851196, 1.5576086044311523, 1.5732736587524414, 1.6007156372070312, 1.5923535823822021, 1.5888965129852295, 1.6030343770980835, 1.621445655822754, 1.6184462308883667, 1.6212974786758423, 1.6412006616592407, 1.6374123096466064, 1.6485450267791748, 1.6649956703186035, 1.6440560817718506, 1.6952407360076904, 1.6891950368881226, 1.691792607307434, 1.7142939567565918, 1.7350116968154907, 1.7310389280319214, 1.7280213832855225, 1.7387698888778687, 1.754279613494873, 1.7621891498565674, 1.7864680290222168, 1.787689208984375, 1.8167721033096313, 1.7968332767486572, 1.8029118776321411, 1.8430875539779663, 1.8444241285324097, 1.8682667016983032, 1.8674299716949463, 1.8694941997528076, 1.9002596139907837, 1.9038031101226807, 1.9179506301879883, 1.9282459020614624, 1.9409689903259277, 1.9488096237182617, 1.9541680812835693, 1.954360008239746, 1.9995976686477661, 1.9950610399246216, 2.0060603618621826, 2.026125907897949, 2.0465927124023438, 2.0536468029022217, 2.075172185897827, 2.087938070297241, 2.1183559894561768, 2.1361641883850098, 2.153150796890259, 2.154803991317749, 2.1757702827453613, 2.1913695335388184, 2.214547634124756, 2.232283353805542, 2.238434076309204, 2.2688188552856445, 2.2896084785461426, 2.2941761016845703, 2.319500207901001, 2.350283145904541, 2.367222547531128, 2.407142400741577, 2.41799259185791, 2.4298667907714844, 2.4617011547088623, 2.482404947280884, 2.5033397674560547, 2.5237157344818115, 2.5500004291534424, 2.580537796020508, 2.6048519611358643, 2.622532367706299, 2.6476290225982666, 2.6880431175231934, 2.7213940620422363, 2.7643232345581055, 2.78629732131958, 2.806502103805542, 2.825786590576172, 2.850628614425659, 2.8622803688049316, 2.8938181400299072, 2.928983688354492, 2.9871532917022705, 3.020090341567993, 3.0538618564605713, 3.0818138122558594, 3.085261106491089, 3.1445932388305664, 3.1876230239868164, 3.215151786804199, 3.239553689956665, 3.286069631576538, 3.323265790939331, 3.345794439315796, 3.3814642429351807, 3.4008073806762695, 3.4568099975585938, 3.475022554397583, 3.492178201675415, 3.5350289344787598, 3.590928792953491, 3.634587526321411, 3.6340298652648926, 3.6833975315093994, 3.7282252311706543, 3.7698464393615723, 3.7970118522644043, 3.8424956798553467, 3.885333776473999, 3.916003704071045, 3.966566324234009, 4.012039661407471, 4.059783458709717, 4.089804649353027, 4.150440692901611, 4.201041221618652, 4.22130012512207, 4.279175281524658, 4.347393035888672, 4.374120235443115, 4.455979824066162, 4.518765926361084, 4.599409580230713, 4.873114109039307, 8.081011772155762, 8.033185005187988, 8.01553726196289, 8.006645202636719, 7.982000350952148, 7.961311340332031, 7.945826053619385, 7.913276672363281, 7.871557712554932, 7.873652935028076, 7.850883483886719, 7.823779582977295, 7.801501274108887, 7.800025463104248, 7.759693145751953, 7.762657642364502, 7.7084455490112305, 7.709400653839111, 7.706444263458252, 7.673673629760742, 7.669826984405518, 7.640074253082275, 7.630077838897705, 7.5903639793396, 7.57861328125, 7.561874866485596, 7.531800746917725, 7.5181145668029785, 7.505020618438721, 7.490906715393066, 7.478100299835205, 7.4589715003967285, 7.432025909423828, 7.43917989730835, 7.398972511291504, 7.4113922119140625, 7.375154972076416, 7.3252482414245605, 7.281563758850098, 7.256998062133789, 7.206136226654053, 7.158998489379883, 7.128703594207764, 7.087995529174805, 7.065366268157959, 7.027129650115967, 6.981722354888916, 6.958482265472412, 6.916148662567139, 6.883966445922852, 6.864260196685791, 6.823997974395752, 6.7837018966674805, 6.764318466186523, 6.723448753356934, 6.700249195098877, 6.6696600914001465, 6.635529518127441, 6.6010541915893555, 6.566425800323486, 6.549598693847656, 6.525051116943359, 6.492159843444824, 6.463655948638916, 6.429729461669922, 6.4159255027771, 6.389779090881348, 6.360184669494629, 6.323994159698486, 6.291022300720215, 6.274383544921875, 6.233654022216797, 6.219995498657227, 6.209668159484863, 6.16991662979126, 6.161260604858398, 6.117824077606201, 6.119492530822754, 6.09412145614624, 6.055784225463867, 6.044925212860107, 6.016813278198242, 5.988968849182129, 5.97188138961792, 5.963805675506592, 5.933463096618652, 5.9063591957092285, 5.882022857666016, 5.871691703796387, 5.857757091522217, 5.817998886108398, 5.797792911529541, 5.776443958282471, 5.77714729309082, 5.7514190673828125, 5.734630107879639, 5.7150797843933105, 5.682445526123047, 5.664013862609863, 5.660037040710449, 5.634123802185059, 5.616971492767334, 5.59607458114624, 5.596613883972168, 5.563606262207031, 5.549274921417236, 5.554925441741943, 5.521739482879639, 5.511739730834961, 5.491610050201416, 5.4576497077941895, 5.463366508483887, 5.452814102172852, 5.433581829071045, 5.415577411651611, 5.401612281799316, 5.3839850425720215, 5.366507530212402, 5.385624408721924, 5.353959560394287, 5.334810733795166, 5.335477828979492, 5.295341968536377, 5.2735066413879395, 5.2781147956848145, 5.26279354095459, 5.239749431610107, 5.234755039215088, 5.2304816246032715, 5.219087600708008, 5.156981945037842, 5.127668380737305, 5.046020984649658, 4.973938465118408, 4.929352760314941, 4.881046772003174, 4.83482551574707, 4.770086288452148, 4.71209716796875, 4.643720626831055, 4.599474906921387, 4.564696311950684, 4.5285820960998535, 4.466608047485352, 4.433911323547363, 4.364446640014648, 4.320850372314453, 4.281319618225098, 4.228600025177002, 4.168078422546387, 4.134126663208008, 4.100247859954834, 4.054011344909668, 4.024130821228027, 3.9699628353118896, 3.9264612197875977, 3.9101827144622803, 3.855966806411743, 3.8216984272003174, 3.7910704612731934, 3.754396677017212, 3.7247376441955566, 3.679821491241455, 3.638929843902588, 3.6136608123779297, 3.589632987976074, 3.552496910095215, 3.4954121112823486, 3.481807231903076, 3.4581310749053955, 3.443189859390259, 3.404249906539917, 3.372771739959717, 3.3389892578125, 3.318854570388794, 3.294741153717041, 3.256934881210327, 3.241410732269287, 3.206129312515259, 3.183703660964966, 3.159186363220215, 3.132725238800049, 3.1177220344543457, 3.08353328704834, 3.064840078353882, 3.060314416885376, 3.0228793621063232, 3.0028042793273926, 2.9877166748046875, 2.956775188446045, 2.927612781524658, 2.9228670597076416, 2.913632869720459, 2.881608724594116, 2.8507239818573, 2.863251209259033, 2.83134126663208, 2.8265392780303955, 2.7800655364990234, 2.790221929550171, 2.7622644901275635, 2.7388553619384766, 2.733018636703491, 2.701937675476074, 2.6897735595703125, 2.668947696685791, 2.649932861328125, 2.636601209640503, 2.6257448196411133, 2.600614309310913, 2.5938034057617188, 2.586456060409546, 2.5687754154205322, 2.550889492034912, 2.5451819896698, 2.522045135498047, 2.4949769973754883, 2.494562864303589, 2.4946863651275635, 2.4642817974090576, 2.4562880992889404, 2.4361488819122314, 2.4390642642974854, 2.426819086074829, 2.4079580307006836, 2.4041545391082764, 2.4184696674346924, 2.368490219116211, 2.376157760620117, 2.3510475158691406, 2.3445804119110107, 2.326807975769043, 2.335411787033081, 2.3205227851867676, 2.296910524368286, 2.3037948608398438, 2.307281255722046, 2.289297103881836, 2.269857168197632, 2.2581775188446045, 2.230308771133423, 2.218533515930176, 2.230478286743164, 2.220280170440674, 2.1978368759155273, 2.1773128509521484, 2.194159746170044, 2.18588924407959, 2.168574571609497, 2.158554792404175, 2.154024839401245, 2.150851011276245, 2.121788501739502, 2.117682695388794, 2.130072832107544, 2.1195125579833984, 2.110980987548828, 2.1042516231536865, 2.1096160411834717, 2.0841100215911865, 2.0890042781829834, 2.0848655700683594, 2.077897310256958, 2.0597686767578125, 2.0425546169281006, 2.0473833084106445, 2.04007625579834, 2.0247673988342285, 2.0135912895202637, 2.0246336460113525, 1.994181752204895, 2.0011956691741943, 2.0080206394195557, 2.000357151031494, 2.001927375793457, 1.991909384727478, 1.978611707687378, 1.9671690464019775, 1.9564732313156128, 1.9667388200759888, 1.9385260343551636, 1.9539562463760376, 1.9624476432800293, 1.9445561170578003, 1.9561294317245483, 1.9290833473205566, 1.929966688156128, 1.9065461158752441, 1.9207451343536377, 1.9042049646377563, 1.9149340391159058, 1.9192031621932983, 1.8985371589660645, 1.8915488719940186, 1.895458698272705, 1.878476858139038, 1.863065481185913, 1.8926254510879517, 1.8694205284118652, 1.8654720783233643, 1.8661669492721558, 1.8628134727478027, 1.8534328937530518, 1.853511929512024, 1.8413550853729248, 1.8473050594329834, 1.8143155574798584, 1.8465702533721924, 1.8355233669281006, 1.8207069635391235, 1.8224455118179321, 1.8301048278808594, 1.8227698802947998, 1.8161507844924927, 1.8215336799621582, 1.7832871675491333, 1.7942627668380737, 1.800428867340088, 1.7904040813446045, 1.7888946533203125, 1.7835900783538818, 1.7886230945587158, 1.776021122932434, 1.7877559661865234, 1.7813985347747803, 1.7751659154891968, 1.7528626918792725, 1.7821303606033325, 1.7736083269119263, 1.7757091522216797, 1.7727446556091309, 1.7703404426574707, 1.7481794357299805, 1.7389355897903442, 1.737409234046936, 1.755173921585083, 1.7489337921142578, 1.7343677282333374, 1.7321546077728271, 1.72077214717865, 1.7393536567687988, 1.7535864114761353, 1.7345038652420044, 1.7184196710586548, 1.7187702655792236, 1.726027488708496, 1.7293686866760254, 1.7402448654174805, 1.7330480813980103, 1.7197211980819702, 1.7034670114517212, 1.7351081371307373, 1.702742576599121, 1.728715181350708, 1.71047043800354, 1.6912294626235962, 1.716569185256958, 1.7024551630020142, 1.7083841562271118, 1.7260808944702148, 1.7074670791625977, 1.7141796350479126, 1.698341727256775, 1.6990808248519897, 1.690913438796997, 1.6984832286834717, 1.6988661289215088, 1.7087466716766357, 1.7008533477783203, 1.7037696838378906, 1.7018053531646729, 1.68166983127594, 1.7024885416030884, 1.6843293905258179, 1.6900957822799683, 1.7017353773117065, 1.6878305673599243, 1.714671015739441, 1.6864593029022217, 1.6928365230560303, 1.6922880411148071, 1.6946486234664917, 1.6955534219741821, 1.6833845376968384, 1.68863046169281, 1.7092479467391968, 1.6782454252243042, 1.7172389030456543, 1.708162784576416, 1.677870750427246, 1.703518271446228, 1.676746129989624, 1.6906845569610596, 1.6863490343093872, 1.6891030073165894, 1.691182017326355, 1.6851692199707031, 1.712912678718567, 1.6919856071472168, 1.701343059539795, 1.6836133003234863, 1.6985173225402832, 1.6899797916412354, 1.687749981880188, 1.694572925567627, 1.6927080154418945, 1.7172040939331055, 1.691721796989441, 1.7006046772003174, 1.6844815015792847, 1.6851774454116821, 1.7056851387023926, 1.6952621936798096, 1.7059407234191895, 1.7053899765014648, 1.7220276594161987, 1.693112850189209, 1.6949782371520996, 1.7058112621307373, 1.6919444799423218, 1.7057456970214844, 1.727709412574768, 1.706856369972229, 1.7062273025512695, 1.6996322870254517, 1.7135045528411865, 1.707441806793213, 1.7042276859283447, 1.6965895891189575, 1.708519697189331, 1.7159168720245361, 1.7242646217346191, 1.7202234268188477, 1.7223777770996094, 1.7238636016845703, 1.72704017162323, 1.7486944198608398, 1.7240298986434937, 1.7350387573242188, 1.731045126914978, 1.7362463474273682]
angles = np.zeros(len(ranges))
locations = np.zeros((len(ranges),2))

#GET THE ANGLES
for i in range(len(ranges)):
    increment = 0.00436331471428
    angles[i] = -2.3561899662 + increment * i

start = time.time()

#GET THE CARTESIAN POSITIONs
for i in range(len(ranges)):
    locations[i,0] = ranges[i] * math.cos(angles[i])
    locations[i,1] = ranges[i] * math.sin(angles[i])

#KMEANS
# n = 2
# km = KMeans(n_clusters=n, random_state=30)
# km.fit(locations)
# prediction = km.labels_
# centers = km.cluster_centers_
# print(centers)

#COMPUTE DBSCAN
db = DBSCAN(eps=0.3, min_samples=2).fit(locations)
prediction = db.labels_


#FIND THE MEDIAN POINT
xs = [x[0] for x in locations]
ys = [y[1] for y in locations]
x_median=sum(xs)/len(xs)
y_median=sum(ys)/len(ys)

print('Time spent on this code: %.2f' %(time.time()-start))

#PLOT
color_list = ['C0','C1','C2','C3','C4','C5','C6','C7','C8','C9']
color = [color_list[i] for i in prediction]
plt.scatter(xs, ys, c= color)
plt.scatter(0,0,c="C3")
plt.scatter(x_median,y_median,c="C4")
# plt.scatter([centers[0,0],centers[1,0]],[centers[0,1],centers[1,1]],c="C5")
plt.show()

