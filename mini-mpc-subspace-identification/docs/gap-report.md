# Gap Report: Subspace Identification

## Missing Items
1. L7: Additional vendor wrappers (Shell SMOC, ABB Predict & Control)
2. L8: MOESP implementation separate from N4SID
3. L8: CVA implementation separate from N4SID
4. L7: OSIsoft PI data interface integration

## Priority
- High: Separate MOESP/CVA implementations for algorithm comparison
- Medium: Additional industrial vendor configurations
- Low: Real-time data historian integration

## Known Limitations
- SVD uses Jacobi method (accurate but O(n^3), not scalable beyond ~100x100)
- No LAPACK/BLAS integration (pure C for portability)
- Recursive N4SID is sliding-window re-identification, not incremental SVD update
