#include "marginalization_factor.h"

void ResidualBlockInfo::Evaluate()
{
    residuals.resize(cost_function->num_residuals());
    std::vector<int> block_sizes = cost_function->parameter_block_sizes();
    raw_jacobians = new double*[block_sizes.size()];
    jacobians.resize(block_sizes.size());

    for(int i = 0; i < static_cast<int>(block_sizes.size()); i++)
    {
        jacobians[i].resize(cost_function->num_residuals(),block_sizes[i]);
        raw_jacobians[i] = jacobians[i].data();
    }
    cost_function->Evaluate(parameter_blocks.data(),residuals.data(),raw_jacobians);


    if (loss_function)
    {
        double residual_scaling_, alpha_sq_norm_;

        double sq_norm, rho[3];

        sq_norm = residuals.squaredNorm();
        loss_function->Evaluate(sq_norm, rho);
        //printf("sq_norm: %f, rho[0]: %f, rho[1]: %f, rho[2]: %f\n", sq_norm, rho[0], rho[1], rho[2]);

        double sqrt_rho1_ = sqrt(rho[1]);

        if ((sq_norm == 0.0) || (rho[2] <= 0.0))
        {
            residual_scaling_ = sqrt_rho1_;
            alpha_sq_norm_ = 0.0;
        }
        else
        {
            const double D = 1.0 + 2.0 * sq_norm * rho[2] / rho[1];
            const double alpha = 1.0 - sqrt(D);
            residual_scaling_ = sqrt_rho1_ / (1 - alpha);
            alpha_sq_norm_ = alpha / sq_norm;
        }

        for (int i = 0; i < static_cast<int>(parameter_blocks.size()); i++)
        {
            jacobians[i] = sqrt_rho1_ * (jacobians[i] - alpha_sq_norm_ * residuals * (residuals.transpose() * jacobians[i]));
        }

        residuals *= residual_scaling_;
    }

}

void MarginalizationInfo::addResidualBlockInfo(ResidualBlockInfo* residual_block_info)
{
    factors.emplace_back(residual_block_info);
    std::vector<double*>& parameter_blocks = residual_block_info->parameter_blocks;
    std::vector<int> parameter_block_sizes = residual_block_info->cost_function->parameter_block_sizes();
    
    for(int i = 0; i < static_cast<int>(residual_block_info->parameter_blocks.size()); i++)
    {
        double* addr = parameter_blocks[i];
        int size = parameter_block_sizes[i];
        parameter_block_size[reinterpret_cast<long>(addr)] = size;
    }
    
    for(int i = 0; i < static_cast<int>(residual_block_info->drop_set.size()); i++)
    {
        double* addr = parameter_blocks[residual_block_info->drop_set[i]];
        parameter_block_idx[reinterpret_cast<long>(addr)] = 0;
    }
}

void MarginalizationInfo::preMarginalize()
{
    for(auto it : factors)
    {
        it->Evaluate();

        std::vector<int> block_sizes = it->cost_function->parameter_block_sizes();
        for(int i = 0; i < static_cast<int>(block_sizes.size()); i++)
        {
            long addr = reinterpret_cast<long>(it->parameter_blocks[i]);
            int size = block_sizes[i];
            if(parameter_block_data.find(addr) == parameter_block_data.end())
            {
                double* data = new double[size];
                memcpy(data,it->parameter_blocks[i],sizeof(double)*size);
                parameter_block_data[addr] = data;
            }
        }
    }
}

void* ThreadConstructA(void* threadsstruct)
{
    ThreadsStruct* p = ((ThreadsStruct*)threadsstruct);
    for (auto it : p->sub_factors)
    {
        for (int i = 0; i < static_cast<int>(it->parameter_blocks.size()); i++)
        {
            int idx_i = p->parameter_block_idx[reinterpret_cast<long>(it->parameter_blocks[i])];
            int size_i = p->parameter_block_size[reinterpret_cast<long>(it->parameter_blocks[i])];
            Eigen::MatrixXd jacobian_i = it->jacobians[i];
            for (int j = i; j < static_cast<int>(it->parameter_blocks.size()); j++)
            {
                int idx_j = p->parameter_block_idx[reinterpret_cast<long>(it->parameter_blocks[j])];
                int size_j = p->parameter_block_size[reinterpret_cast<long>(it->parameter_blocks[j])];
                Eigen::MatrixXd jacobian_j = it->jacobians[j];
                if (i == j)
                    p->A.block(idx_i, idx_j, size_i, size_j) += jacobian_i.transpose() * jacobian_j;
                else
                {
                    p->A.block(idx_i, idx_j, size_i, size_j) += jacobian_i.transpose() * jacobian_j;
                    p->A.block(idx_j, idx_i, size_j, size_i) = p->A.block(idx_i, idx_j, size_i, size_j).transpose();
                }
            }
            p->b.segment(idx_i, size_i) += jacobian_i.transpose() * it->residuals;
        }
    }
    return threadsstruct;
}

void MarginalizationInfo::marginalize()
{
    int pos = 0;
    for(auto& it : parameter_block_idx)
    {
        it.second = pos;
        pos += parameter_block_size[it.first];
    }
    m = pos;
    for(const auto& it : parameter_block_size)
    {
        if(parameter_block_idx.find(it.first) == parameter_block_idx.end())
        {
            parameter_block_idx[it.first] = pos;
            pos += it.second;
        }
    }
    n = pos - m;

    Eigen::MatrixXd A(pos,pos);
    Eigen::VectorXd b(pos);
    A.setZero();
    b.setZero();

    printf("marg factors num: %d\n",factors.size());

    // pthread_t tids[4];
    // ThreadsStruct threadsstruct[4];
    // int i = 0;
    // for(auto it : factors)
    // {
    //     threadsstruct[i].sub_factors.push_back(it);
    //     i++;
    //     i = i % 2;
    // }
    // for( int i = 0; i < 2; i++)
    // {
    //     threadsstruct[i].A = Eigen::MatrixXd::Zero(pos,pos);
    //     threadsstruct[i].b = Eigen::VectorXd::Zero(pos);
    //     threadsstruct[i].parameter_block_size = parameter_block_size;
    //     threadsstruct[i].parameter_block_idx = parameter_block_idx;
    //     int ret = pthread_create(&tids[i],NULL,ThreadConstructA,(void*)&(threadsstruct[i]));
    //     if (ret != 0)
    //     {
    //         ROS_WARN("pthread_create error");
    //         ROS_BREAK();
    //     }
    // }
    // for( int i = 2 - 1; i >= 0; i--)  
    // {
    //     pthread_join( tids[i], NULL ); 
    //     A += threadsstruct[i].A;
    //     b += threadsstruct[i].b;
    // }

    for(auto it : factors)
    {
        for(int i = 0; i < static_cast<int>(it->parameter_blocks.size()); i++)
        {
            int idx_i = parameter_block_idx[reinterpret_cast<long>(it->parameter_blocks[i])];
            int size_i = parameter_block_size[reinterpret_cast<long>(it->parameter_blocks[i])];
            Eigen::MatrixXd jacobian_i = it->jacobians[i];
            for(int j = i; j < static_cast<int>(it->parameter_blocks.size()); j++)
            {
                int idx_j = parameter_block_idx[reinterpret_cast<long>(it->parameter_blocks[j])];
                int size_j = parameter_block_size[reinterpret_cast<long>(it->parameter_blocks[j])];
                Eigen::MatrixXd jacobian_j = it->jacobians[j];
                if(i == j)
                    A.block(idx_i,idx_j,size_i,size_j) += jacobian_i.transpose() * jacobian_j;
                else
                {
                    A.block(idx_i,idx_j,size_i,size_j) += jacobian_i.transpose() * jacobian_j;
                    A.block(idx_j,idx_i,size_j,size_i) = A.block(idx_i,idx_j,size_i,size_j).transpose();
                    //A.block(idx_j,idx_i,size_j,size_i) += jacobian_j.transpose() * jacobian_i;
                }
            }
            b.segment(idx_i,size_i) += jacobian_i.transpose() * it->residuals;
        }
    }

    Eigen::MatrixXd Amm = 0.5 * (A.block(0,0,m,m) + A.block(0,0,m,m).transpose());
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes(Amm);
    Eigen::MatrixXd Amm_inv = saes.eigenvectors() * Eigen::VectorXd((saes.eigenvalues().array()>eps).select(
        saes.eigenvalues().array().inverse(),0)).asDiagonal()*saes.eigenvectors().transpose();
    Eigen::VectorXd bmm = b.segment(0, m);
    Eigen::MatrixXd Amr = A.block(0, m, m, n);
    Eigen::MatrixXd Arm = A.block(m, 0, n, m);
    Eigen::MatrixXd Arr = A.block(m, m, n, n);
    Eigen::VectorXd brr = b.segment(m, n);
    A = Arr - Arm * Amm_inv * Amr;
    b = brr - Arm * Amm_inv * bmm;

    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes2(A);
    Eigen::VectorXd S = Eigen::VectorXd((saes2.eigenvalues().array() > eps).select(saes2.eigenvalues().array(), 0));
    Eigen::VectorXd S_inv = Eigen::VectorXd((saes2.eigenvalues().array() > eps).select(saes2.eigenvalues().array().inverse(), 0));

    Eigen::VectorXd S_sqrt = S.cwiseSqrt();
    Eigen::VectorXd S_inv_sqrt = S_inv.cwiseSqrt();

    linearized_jacobians = S_sqrt.asDiagonal() * saes2.eigenvectors().transpose();
    linearized_residuals = S_inv_sqrt.asDiagonal() * saes2.eigenvectors().transpose() * b;
}

std::vector<double*> MarginalizationInfo::getParameterBlocks(std::unordered_map<long,double*>& addr_shift)
{
    std::vector<double*> keep_block_addr;
    keep_block_size.clear();
    keep_block_idx.clear();
    keep_block_data.clear();
    for(const auto& it : parameter_block_idx)
    {
        if(it.second >= m)
        {
            keep_block_size.push_back(parameter_block_size[it.first]);
            keep_block_idx.push_back(parameter_block_idx[it.first]);
            keep_block_data.push_back(parameter_block_data[it.first]);
            keep_block_addr.push_back(addr_shift[it.first]);
        }
    }
    sum_block_size = std::accumulate(std::begin(keep_block_size),std::end(keep_block_size),0);
    return keep_block_addr;
}

MarginalizationFactor::MarginalizationFactor(MarginalizationInfo* _marginalization_info):marginalization_info(_marginalization_info)
{
    int cnt = 0;
    for (auto it : marginalization_info->keep_block_size)
    {
        mutable_parameter_block_sizes()->push_back(it);
        cnt += it;
    }
    //printf("residual size: %d, %d\n", cnt, n);
    set_num_residuals(marginalization_info->n);
};

bool MarginalizationFactor::Evaluate(double const* const* parameters, double* residuals, double** jacobians) const
{
    int n = marginalization_info->n;
    int m = marginalization_info->m;
    Eigen::VectorXd dx(n);
    for(int i = 0; i < static_cast<int>(marginalization_info->keep_block_size.size()); i++)
    {
        int size = marginalization_info->keep_block_size[i];
        int idx = marginalization_info->keep_block_idx[i] - m;
        Eigen::VectorXd x = Eigen::Map<const Eigen::VectorXd>(parameters[i],size);
        Eigen::VectorXd x0 = Eigen::Map<const Eigen::VectorXd>(marginalization_info->keep_block_data[i],size);
        if(size != 3)
            dx.segment(idx,size) = x - x0;
        else
        {
            dx.segment<2>(idx) = x.head(2) - x0.head(2);
            dx(idx+2) = Utility::normalizeAngle(x(2)-x0(2));
        }
    }
    Eigen::Map<Eigen::VectorXd>(residuals,n) = marginalization_info->linearized_residuals + marginalization_info->linearized_jacobians * dx;
    if (jacobians)
    {

        for (int i = 0; i < static_cast<int>(marginalization_info->keep_block_size.size()); i++)
        {
            if (jacobians[i])
            {
                int size = marginalization_info->keep_block_size[i];
                int idx = marginalization_info->keep_block_idx[i] - m;
                Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> jacobian(jacobians[i], n, size);
                jacobian.setZero();
                jacobian = marginalization_info->linearized_jacobians.middleCols(idx, size);
            }
        }
    }
    return true;
}

