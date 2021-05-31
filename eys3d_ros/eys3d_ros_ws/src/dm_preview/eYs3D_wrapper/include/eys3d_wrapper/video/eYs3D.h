/*
 * Copyright (C) 2015-2019 ICL/ITRI
 * All rights reserved.
 *
 * NOTICE:  All information contained herein is, and remains
 * the property of ICL/ITRI and its suppliers, if any.
 * The intellectual and technical concepts contained
 * herein are proprietary to ICL/ITRI and its suppliers and
 * may be covered by Taiwan and Foreign Patents,
 * patents in process, and are protected by trade secret or copyright law.
 * Dissemination of this information or reproduction of this material
 * is strictly forbidden unless prior written permission is obtained
 * from ICL/ITRI.
 */
 
#pragma onceeYs3D

#define eYs3D_API_MAJOR_VERSION    0
#define eYs3D_API_MINOR_VERSION    11
#define eYs3D_API_PATCH_VERSION    0
#define eYs3D_API_BUILD_VERSION    0

#ifndef STRINGIFY
#define STRINGIFY(arg) #arg
#endif
#ifndef VAR_ARG_STRING
#define VAR_ARG_STRING(arg) STRINGIFY(arg)
#endif

/* Versioning rules:
 *     For each release at least one of [MJR/MNR/PTCH] triple is promoted
 *     Versions that differ by RS2_API_PATCH_VERSION only are interface-compatible,
 *         i.e. no user-code changes required
 *     Versions that differ by MAJOR/MINOR VERSION component can introduce API changes
 *
 * Version in encoded integer format (1,9,x) -> 01090x. note that each component is limited into [0-99] range by design
 */
#define eYs3D_API_VERSION (((eYs3D_API_MAJOR_VERSION) * 10000) + ((eYs3D_API_MINOR_VERSION) * 100) + (eYs3D_API_PATCH_VERSION))

/* Return version in "X.Y.Z" format */
#define eYs3D_API_VERSION_STR (VAR_ARG_STRING(eYs3D_API_MAJOR_VERSION.eYs3D_API_MINOR_VERSION.eYs3D_API_PATCH_VERSION))
#define eYs3D_API_FULL_VERSION_STR (VAR_ARG_STRING(eYs3D_API_MAJOR_VERSION.eYs3D_API_MINOR_VERSION.eYs3D_API_PATCH_VERSION.eYs3D_API_BUILD_VERSION))
