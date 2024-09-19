## =========================================================================
## @author Leonardo Florez-Valencia (florez-l@javeriana.edu.co)
## =========================================================================
## https://www.prinmath.com/csci5229/OBJ/index.html
## =========================================================================

import heapq, math, sys

'''
'''
def ReadOBJ( fname ):
  V = []
  A = {}
  fstr = open( fname )
  for line in fstr:
    d = line.rstrip( ).lstrip( ).split( ' ' )
    if d[ 0 ] == 'v':
      V += [ tuple( [ float( c ) for c in d[ 1 : ] ] ) ]
    elif d[ 0 ] == 'f':
      I = [ int( i.split( '/' )[ 0 ] ) - 1 for i in d[ 1 : ] ]
      for i in range( len( I ) ):
        s = I[ i ]
        e = I[ ( i + 1 ) % len( I ) ]
        if not s in A:
          A[ s ] = []
        # end if
        if not e in A:
          A[ e ] = []
        # end if
        A[ s ] += [ e ]
        A[ e ] += [ s ]
      # end for
    # end if
  # end for
  fstr.close( )

  return ( V, A )
# end def

'''
'''
def FormatPathAsOBJString( V, P ):
  s = ''
  for i in P:
    s += 'v ' + str( V[ i ][ 0 ] )
    s +=  ' ' + str( V[ i ][ 1 ] )
    s +=  ' ' + str( V[ i ][ 2 ] )
    s += '\n'
  # end for

  s += '\nl'
  for i in range( len( P ) ):
    s += ' ' + str( i + 1 )
  # end for

  return s
# end def

'''
'''
def Distance( a, b ):
  s = 0
  for i in range( len( a ) ):
    s += ( a[ i ] - b[ i ] ) ** 2
  # end for
  return s ** 0.5
# end def

'''
'''
def Kruskal( G ):

  V, A = G

  # First edge
  e = ( math.inf, -1, -1 )
  for a in A:
    for n in A[ a ]:
      d = Distance( V[ a ], V[ n ] )
      if d < e[ 0 ]:
        e = ( d, a, n )
      # end if
    # end for 
  # end for

  # Initialize queue, marks and MST
  Q = [ e ]
  M = [ False for i in range( len( V ) ) ]
  T = [ i for i in range( len( V ) ) ]

  # Main loop
  start = True
  while len( Q ) > 0:
    d, i, j = heapq.heappop( Q )

    if start:
      M[ i ] = True
      M[ j ] = True
      start = False

      for k in A[ i ]:
        heapq.heappush( Q, ( Distance( V[ i ], V[ k ] ), i, k ) )
      # end for

      for k in A[ j ]:
        heapq.heappush( Q, ( Distance( V[ j ], V[ k ] ), j, k ) )
      # end for

    else:
      if M[ i ] and not M[ j ]:
        M[ j ] = True
        T[ j ] = i

        for k in A[ j ]:
          heapq.heappush( Q, ( Distance( V[ j ], V[ k ] ), j, k ) )
        # end for
      # end if
    # end if
  # end while

  return T
# end def

'''
TODO
'''
def Dijkstra( G, s ):
  return None
# end def

'''
'''
def SpanningTree_Backtrack( T, e ):
  P = []
  j = e
  while T[ j ] != j:
    P = [ j ] + P
    j = T[ j ]
  # end while
  P = [ j ] + P
  return P
# end def

'''
TODO
'''
def ChooseMaxPath( G, T ):
  return None
# end def

'''
---------------------------------------------------------------------------
'''
if __name__ == '__main__':

  G = ReadOBJ( sys.argv[ 1 ] )

  K = Kruskal( G )
  KP = ChooseMaxPath( G, K )
  KS = FormatPathAsOBJString( G[ 0 ], KP )

  D = Dijkstra( G, KP[ 0 ] )
  DP = ChooseMaxPath( G, D )
  DS = FormatPathAsOBJString( G[ 0 ], DP )

  fstr = open( 'kruskal.obj', 'w' )
  fstr.write( KS )
  fstr.close( )

  fstr = open( 'dijkstra.obj', 'w' )
  fstr.write( DS )
  fstr.close( )

# end if

## eof - ComputeCutPaths.py
