import matplotlib
import matplotlib.pyplot as plt
matplotlib.rcParams['pdf.fonttype'] = 42
matplotlib.rcParams['ps.fonttype'] = 42

if __name__ == "__main__":

    a_x = range(1, 50)
    a_y = [0.674597652063, 0.47994815497446913, 0.3587211031527827, 0.37505628028855315, 0.4441278672367998, 0.39136982942282367, 0.33930266384282765, 0.32310382690982453, 0.292688118459703, 0.27636341962987737, 0.2926413020542609, 0.28612740819456356, 0.3019182320796824, 0.3366658119024394, 0.3136027979873262, 0.2966352551971198, 0.24729064822674843, 0.24871836679364748, 0.22395041406157928, 0.21595073802292014, 0.22676402528228434, 0.22291881193255988, 0.20937319727560336, 0.22229061351182328, 0.20125891887960853, 0.22404431777973277, 0.2114004391472472, 0.20622674126886756, 0.2033473505529856, 0.18454270623277239, 0.22308763244712584, 0.21563170423206113, 0.21778874827188754, 0.20077347267711793, 0.19667778562447413, 0.19471457445818732, 0.18850319757156117, 0.2028551180575, 0.2007092487210296, 0.1942269032253961, 0.19336891910646284, 0.17795744881220055, 0.1695250312047799, 0.16362209518223103, 0.1654865120421185, 0.1749876323866703, 0.16324539527345017, 0.1666727685731074, 0.16983527676571963]

    b_x = range(1, 50)
    b_y = [0.5452672156646932, 0.4981057975768136, 0.41670118802782335, 0.35919000573639787, 0.3486575944242595, 0.30421750233505807, 0.34530912247047285, 0.29134780848298447, 0.27498802402989797, 0.29910432227281963, 0.2875324259196603, 0.3407879032792732, 0.3189143519310483, 0.31204961124288616, 0.3073901413153531, 0.2829007764422468, 0.2673659089481376, 0.302913110979979, 0.24912745801799815, 0.2593316408193074, 0.23745384310053905, 0.25407877062789386, 0.24234946054294224, 0.23121717832471783, 0.22441362315010788, 0.23215865219207302, 0.22642104974490465, 0.20180229316231257, 0.2161840884165845, 0.20091323611226172, 0.23448174250874898, 0.196369503668126, 0.21106116968066044, 0.18235245644968323, 0.18118566476573977, 0.1863115517955972, 0.2069208311295514, 0.2000007049873253, 0.19994401810054246, 0.2110082513989923, 0.20093663208145038, 0.17920629993898252, 0.20036340521916154, 0.19387550789406174, 0.20663016143757595, 0.217963504655769, 0.21289234109658461, 0.20773224436732643, 0.1863006930470259]

    plt.plot(a_x, a_y, label="Heuristic Filtering")
    plt.plot(b_x, b_y, label="Original")

    plt.title("Novelty Score Throughout Evolution")
    plt.xlabel("Generations of Novelty Search")
    plt.ylabel("Novelty Score")
    plt.legend()
    plt.show()